package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX44;
import frc.robot.Ports;
import org.littletonrobotics.junction.Logger;

/**
 * Intake with a two-motor pivot (leader + follower) and a single roller motor.
 *
 * <p>All pivot control (Motion Magic, homing, voltage) is sent to the leader only. The follower is
 * put into Follower mode once at startup and mirrors the leader's output in hardware, so the two
 * motors can never fight each other from software.
 */
public class Intake extends SubsystemBase {
  private static final Voltage MAX_VOLTAGE = Volts.of(12.0);

  // Pivot mechanical
  /** Motor rotations per pivot rotation. TODO: set to your actual gear reduction. */
  private static final double PIVOT_REDUCTION = 50.0;

  /**
   * Whether the follower spins the same physical direction as the leader (Aligned) or the opposite
   * (Opposed). Opposed is typical when the two motors face each other on opposite sides of the
   * pivot shaft. TODO: verify on the robot.
   */
  private static final MotorAlignmentValue FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed;

  private static final AngularVelocity MAX_PIVOT_SPEED = KrakenX44.kFreeSpeed.div(PIVOT_REDUCTION);
  private static final Angle POSITION_TOLERANCE = Degrees.of(5);

  // Pivot electrical
  private static final Current PIVOT_STATOR_CURRENT_LIMIT = Amps.of(120);
  private static final Current PIVOT_SUPPLY_CURRENT_LIMIT = Amps.of(35);

  // Homing
  private static final double HOMING_PERCENT_OUTPUT = 0.1;
  /** Measured on the leader only. Retune: two motors share the load, so per-motor current differs. */
  private static final Current HOMING_CURRENT_THRESHOLD = Amps.of(5.3);

  // Roller electrical
  private static final Current ROLLER_STATOR_CURRENT_LIMIT = Amps.of(80);
  private static final Current ROLLER_SUPPLY_CURRENT_LIMIT = Amps.of(35);

  // Pivot gains (Motion Magic, Slot 0)
  private static final double kP = 300.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kV = 0.099;

  public enum Speed {
    STOP(0.0),
    SHOOT(0.55),
    INTAKE(0.85),
    EJECT(-0.6);

    private final double percentOutput;

    private Speed(double percentOutput) {
      this.percentOutput = percentOutput;
    }

    public Voltage voltage() {
      return MAX_VOLTAGE.times(percentOutput);
    }
  }

  public enum Position {
    HOMED(Degrees.of(106)),
    STOWED(Degrees.of(100)),
    INTAKE(Degrees.of(-20)),
    AGITATE(Degrees.of(23));

    private final Angle angle;

    private Position(Angle angle) {
      this.angle = angle;
    }

    public Angle angle() {
      return angle;
    }
  }

  private final TalonFX pivotLeader, pivotFollower, rollerMotor;

  private final VoltageOut pivotVoltageRequest = new VoltageOut(Volts.of(0));
  private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut rollerVoltageRequest = new VoltageOut(Volts.of(0));

  private boolean isHomed = false;

  public Intake() {
    pivotLeader = new TalonFX(Ports.kIntakePivotLeader, Ports.kRoboRioCANBus);
    pivotFollower = new TalonFX(Ports.kIntakePivotFollower, Ports.kRoboRioCANBus);
    rollerMotor = new TalonFX(Ports.kIntakeRollers, Ports.kRoboRioCANBus);

    configurePivotMotors();
    configureRollerMotor();

    // Follower mode persists until another control request is sent to that device.
    // Never call setControl on pivotFollower anywhere else.
    pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), FOLLOWER_ALIGNMENT));

    // Auto-home in simulation
    if (RobotBase.isSimulation()) {
      pivotLeader.setPosition(Position.HOMED.angle());
      pivotFollower.setPosition(Position.HOMED.angle());
      isHomed = true;
    }

    SmartDashboard.putData(this);
  }

  private void configurePivotMotors() {
    // Both pivot motors get the same config. Direction between them is handled by
    // FOLLOWER_ALIGNMENT, so don't flip the invert on one of them.
    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(PIVOT_STATOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(PIVOT_SUPPLY_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(PIVOT_REDUCTION))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(MAX_PIVOT_SPEED)
                    .withMotionMagicAcceleration(MAX_PIVOT_SPEED.per(Second)))
            .withSlot0(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKV(kV));

    pivotLeader.getConfigurator().apply(config);
    pivotFollower.getConfigurator().apply(config);
  }

  private void configureRollerMotor() {
    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(ROLLER_STATOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ROLLER_SUPPLY_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true));
    rollerMotor.getConfigurator().apply(config);
  }

  // ---------------------------------------------------------------------------
  // Low-level control
  // ---------------------------------------------------------------------------

  private boolean isPositionWithinTolerance() {
    final Angle currentPosition = pivotLeader.getPosition().getValue();
    final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
    return currentPosition.isNear(targetPosition, POSITION_TOLERANCE);
  }

  private void setPivotPercentOutput(double percentOutput) {
    pivotLeader.setControl(pivotVoltageRequest.withOutput(MAX_VOLTAGE.times(percentOutput)));
  }

  public void set(Position position) {
    pivotLeader.setControl(pivotMotionMagicRequest.withPosition(position.angle()));
  }

  public void set(Speed speed) {
    rollerMotor.setControl(rollerVoltageRequest.withOutput(speed.voltage()));
  }

  /** Stops the rollers. Leaves the pivot where it is. */
  public void stop() {
    set(Speed.STOP);
  }

  public void stopPivot() {
    pivotLeader.setControl(pivotVoltageRequest.withOutput(Volts.of(0)));
    isHomed = false;
  }

  // ---------------------------------------------------------------------------
  // Commands
  // ---------------------------------------------------------------------------

  /** Deploys the pivot and runs the rollers while held; rollers stop on release. */
  public Command intakeCommand() {
    return startEnd(
        () -> {
          set(Position.INTAKE);
          set(Speed.INTAKE);
        },
        () -> set(Speed.STOP));
  }

  /** One-shot version for autos. Does not stop the rollers. */
  public Command intakeAuto() {
    return runOnce(
        () -> {
          set(Position.INTAKE);
          set(Speed.INTAKE);
        });
  }

  public Command intakeCommandNoPivot() {
    return startEnd(() -> set(Speed.INTAKE), () -> set(Speed.STOP));
  }

  public Command ejectCommand() {
    return startEnd(() -> set(Speed.EJECT), () -> set(Speed.STOP));
  }

  public Command stowCommand() {
    return runOnce(
        () -> {
          set(Position.STOWED);
          set(Speed.STOP);
        });
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public Command stopPivotCommand() {
    return runOnce(this::stopPivot);
  }

  /** Bounces the pivot between AGITATE and INTAKE while the rollers run, to unjam game pieces. */
  public Command agitateCommand() {
    return runOnce(() -> set(Speed.SHOOT))
        .andThen(
            Commands.sequence(
                    runOnce(() -> set(Position.AGITATE)),
                    Commands.waitUntil(this::isPositionWithinTolerance),
                    runOnce(() -> set(Position.INTAKE)),
                    Commands.waitUntil(this::isPositionWithinTolerance))
                .repeatedly())
        .handleInterrupt(
            () -> {
              set(Position.INTAKE);
              set(Speed.STOP);
            });
  }

  public Command rollCommand() {
    return runOnce(() -> set(Speed.INTAKE)).handleInterrupt(() -> set(Speed.STOP));
  }

  /** Drives the pivot into its hard stop, zeroes both motors there, then moves to STOWED. */
  public Command homingCommand() {
    return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(HOMING_PERCENT_OUTPUT)),
            Commands.waitUntil(
                () -> pivotLeader.getSupplyCurrent().getValue().gt(HOMING_CURRENT_THRESHOLD)),
            runOnce(
                () -> {
                  pivotLeader.setPosition(Position.HOMED.angle());
                  pivotFollower.setPosition(Position.HOMED.angle());
                  isHomed = true;
                  set(Position.STOWED);
                }))
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  // ---------------------------------------------------------------------------
  // Periodic / telemetry
  // ---------------------------------------------------------------------------

  private void logMotor(String prefix, TalonFX motor) {
    Logger.recordOutput(prefix + "/SupplyCurrentAmps", motor.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput(prefix + "/StatorCurrentAmps", motor.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput(prefix + "/AppliedVoltage", motor.getMotorVoltage().getValue().in(Volts));
    Logger.recordOutput(prefix + "/TemperatureCelsius", motor.getDeviceTemp().getValue().in(Celsius));
  }

  @Override
  public void periodic() {
    // Pivot
    Logger.recordOutput("Intake/Pivot/AngleDegrees", pivotLeader.getPosition().getValue().in(Degrees));
    Logger.recordOutput(
        "Intake/Pivot/FollowerAngleDegrees", pivotFollower.getPosition().getValue().in(Degrees));
    Logger.recordOutput(
        "Intake/Pivot/TargetAngleDegrees", pivotMotionMagicRequest.getPositionMeasure().in(Degrees));
    logMotor("Intake/Pivot/Leader", pivotLeader);
    logMotor("Intake/Pivot/Follower", pivotFollower);

    // Roller
    Logger.recordOutput("Intake/Roller/VelocityRPM", rollerMotor.getVelocity().getValue().in(RPM));
    logMotor("Intake/Roller", rollerMotor);

    // State
    Logger.recordOutput("Intake/IsHomed", isHomed);
    Logger.recordOutput("Intake/AtSetpoint", isPositionWithinTolerance());
    Logger.recordOutput("Intake/CommandActive", getCurrentCommand() != null);
    if (getCurrentCommand() != null) {
      Logger.recordOutput("Intake/CommandName", getCurrentCommand().getName());
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty(
        "Command",
        () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);
    builder.addDoubleProperty(
        "Angle (degrees)", () -> pivotLeader.getPosition().getValue().in(Degrees), null);
    builder.addDoubleProperty("RPM", () -> rollerMotor.getVelocity().getValue().in(RPM), null);
    builder.addDoubleProperty(
        "Pivot Leader Supply Current",
        () -> pivotLeader.getSupplyCurrent().getValue().in(Amps),
        null);
    builder.addDoubleProperty(
        "Pivot Follower Supply Current",
        () -> pivotFollower.getSupplyCurrent().getValue().in(Amps),
        null);
    builder.addDoubleProperty(
        "Roller Supply Current", () -> rollerMotor.getSupplyCurrent().getValue().in(Amps), null);
  }
}