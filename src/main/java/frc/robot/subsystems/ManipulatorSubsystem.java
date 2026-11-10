package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Ports;
/**
 * Single-jointed manipulator driven by two Kraken X44 motors (TalonFX, Phoenix 6).
 *
 * Conventions:
 *  - Angle 0 = manipulator horizontal (parallel to ground), positive = up.
 *  - All TalonFX closed-loop units are MECHANISM rotations (SensorToMechanismRatio handles gearing).
 *  - Gravity feedforward uses Arm_Cosine, which requires 0 = horizontal.
 *
 * TUNE EVERYTHING in ManipulatorConstants before running on a real robot.
 */
public class ManipulatorSubsystem extends SubsystemBase {

  //  Hardware
  private final TalonFX leader = new TalonFX(Ports.kManipulatorLeader, Ports.kRoboRioCANBus);
  private final TalonFX follower = new TalonFX(Ports.kManipulatorFollower, Ports.kRoboRioCANBus);

  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<Angle> positionSignal = leader.getPosition();
  private final StatusSignal<AngularVelocity> velocitySignal = leader.getVelocity();
  private final StatusSignal<Voltage> voltageSignal = leader.getMotorVoltage();
  private final StatusSignal<Current> currentSignal = leader.getStatorCurrent();

  private Angle targetAngle = Degrees.of(ManipulatorConstants.STARTING_ANGLE_DEG);

  //  Simulation
  private final TalonFXSimState leaderSim = leader.getSimState();
  private final SingleJointedArmSim manipulatorSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44(2),
          ManipulatorConstants.GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(ManipulatorConstants.MANIPULATOR_LENGTH_M, ManipulatorConstants.MANIPULATOR_MASS_KG),
          ManipulatorConstants.MANIPULATOR_LENGTH_M,
          Units.degreesToRadians(ManipulatorConstants.MIN_ANGLE_DEG),
          Units.degreesToRadians(ManipulatorConstants.MAX_ANGLE_DEG),
          true,
          Units.degreesToRadians(ManipulatorConstants.STARTING_ANGLE_DEG));

  private final Mechanism2d mech = new Mechanism2d(2, 2);
  private final MechanismRoot2d root = mech.getRoot("ManipulatorPivot", 1, 0.5);
  private final MechanismLigament2d manipulatorLigament =
      root.append(
          new MechanismLigament2d("Manipulator", ManipulatorConstants.MANIPULATOR_LENGTH_M, 0, 6, new Color8Bit(0, 150, 255)));

  //  Setup
  public ManipulatorSubsystem() {
    configureMotors();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, positionSignal, velocitySignal, voltageSignal, currentSignal);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();

    // Tell the motor where the manipulator physically is at boot.
    leader.setPosition(Degrees.of(ManipulatorConstants.STARTING_ANGLE_DEG));

    SmartDashboard.putData("Manipulator/Mechanism", mech);
  }

  private void configureMotors() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted =
        ManipulatorConstants.INVERT_LEADER
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Gearing: 1 mechanism rotation = GEAR_RATIO motor rotations
    cfg.Feedback.SensorToMechanismRatio = ManipulatorConstants.GEAR_RATIO;

    // Current limits
    cfg.CurrentLimits.StatorCurrentLimit = ManipulatorConstants.STATOR_LIMIT_A;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = ManipulatorConstants.SUPPLY_LIMIT_A;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Software limits (mechanism rotations)
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(ManipulatorConstants.MAX_ANGLE_DEG);
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(ManipulatorConstants.MIN_ANGLE_DEG);

    // PID + feedforward
    cfg.Slot0.kP = ManipulatorConstants.kP;
    cfg.Slot0.kI = ManipulatorConstants.kI;
    cfg.Slot0.kD = ManipulatorConstants.kD;
    cfg.Slot0.kS = ManipulatorConstants.kS;
    cfg.Slot0.kV = ManipulatorConstants.kV;
    cfg.Slot0.kA = ManipulatorConstants.kA;
    cfg.Slot0.kG = ManipulatorConstants.kG;
    cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    cfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    // Motion Magic
    cfg.MotionMagic.MotionMagicCruiseVelocity = ManipulatorConstants.CRUISE_VEL_RPS;
    cfg.MotionMagic.MotionMagicAcceleration = ManipulatorConstants.ACCEL_RPS2;
    cfg.MotionMagic.MotionMagicJerk = ManipulatorConstants.JERK_RPS3;

    applyWithRetry(leader, cfg);
    applyWithRetry(follower, cfg);

    follower.setControl(
        new Follower(
            leader.getDeviceID(),
            ManipulatorConstants.FOLLOWER_OPPOSES_LEADER
                ? MotorAlignmentValue.Opposed
                : MotorAlignmentValue.Aligned));
  }

  private static void applyWithRetry(TalonFX motor, TalonFXConfiguration cfg) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5 && !status.isOK(); i++) {
      status = motor.getConfigurator().apply(cfg);
    }
    if (!status.isOK()) {
      System.err.println(
          "Failed to configure TalonFX " + motor.getDeviceID() + ": " + status.toString());
    }
  }

  //  API
  /** Current manipulator angle (0 = horizontal). */
  public Angle getAngle() {
    return BaseStatusSignal.getLatencyCompensatedValue(positionSignal, velocitySignal);
  }

  public Angle getTarget() {
    return targetAngle;
  }

  public boolean atTarget() {
    return Math.abs(getAngle().in(Degrees) - targetAngle.in(Degrees)) < ManipulatorConstants.TOLERANCE_DEG;
  }

  /** Move to a position using Motion Magic. */
  public void setAngle(Angle angle) {
    double clampedDeg =
        Math.max(ManipulatorConstants.MIN_ANGLE_DEG, Math.min(ManipulatorConstants.MAX_ANGLE_DEG, angle.in(Degrees)));
    targetAngle = Degrees.of(clampedDeg);
    leader.setControl(positionRequest.withPosition(targetAngle));
  }

  public void setVoltage(double volts) {
    leader.setControl(voltageRequest.withOutput(volts));
  }

  public void stop() {
    leader.setControl(voltageRequest.withOutput(0));
  }

  //  Commands
  /** Go to an angle and finish when within tolerance. */
  public Command goTo(Angle angle) {
    return runOnce(() -> setAngle(angle))
        .andThen(run(() -> {}).until(this::atTarget))
        .withName("Manipulator goTo " + (int) angle.in(Degrees));
  }

  /** Go to an angle and hold it until interrupted (good for whileTrue bindings). */
  public Command holdAt(Angle angle) {
    return run(() -> setAngle(angle)).withName("Manipulator holdAt " + (int) angle.in(Degrees));
  }

  /** Manual control: input in [-1, 1], scaled to 6 V. Holds position when released. */
  public Command manual(java.util.function.DoubleSupplier input) {
    return run(() -> {
          double in = input.getAsDouble();
          if (Math.abs(in) > 0.1) {
            setVoltage(in * 6.0);
            targetAngle = getAngle();
          } else {
            setAngle(targetAngle);
          }
        })
        .withName("Manipulator manual");
  }

  /** Hold the manipulator wherever it currently is. */
  public Command holdPosition() {
    return runOnce(() -> setAngle(getAngle())).withName("Manipulator holdPosition");
  }

  /**
   * Move the setpoint at a constant rate (deg/sec) while running, then hold wherever the manipulator is
   * when the command ends (e.g. button released). Positive = up.
   */
  public Command nudge(double degPerSec) {
    return run(() -> setAngle(Degrees.of(targetAngle.in(Degrees) + degPerSec * 0.020)))
        .finallyDo(() -> setAngle(getAngle()))
        .withName("Manipulator nudge");
  }

  // Periodic
  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(positionSignal, velocitySignal, voltageSignal, currentSignal);

    SmartDashboard.putNumber("Manipulator/AngleDeg", positionSignal.getValue().in(Degrees));
    SmartDashboard.putNumber("Manipulator/TargetDeg", targetAngle.in(Degrees));
    SmartDashboard.putBoolean("Manipulator/AtTarget", atTarget());
    SmartDashboard.putNumber("Manipulator/StatorCurrentA", currentSignal.getValueAsDouble());
    SmartDashboard.putNumber("Manipulator/AppliedVolts", voltageSignal.getValueAsDouble());

    manipulatorLigament.setAngle(positionSignal.getValue().in(Degrees));
  }

  @Override
  public void simulationPeriodic() {
    leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Feed motor voltage into the physics sim
    manipulatorSim.setInput(leaderSim.getMotorVoltage());
    manipulatorSim.update(0.020);

    // Feed the result back into the simulated TalonFX (rotor = mechanism * gear ratio)
    double manipulatorRot = Units.radiansToRotations(manipulatorSim.getAngleRads());
    double manipulatorRps = Units.radiansToRotations(manipulatorSim.getVelocityRadPerSec());
    leaderSim.setRawRotorPosition(manipulatorRot * ManipulatorConstants.GEAR_RATIO);
    leaderSim.setRotorVelocity(manipulatorRps * ManipulatorConstants.GEAR_RATIO);
  }
}