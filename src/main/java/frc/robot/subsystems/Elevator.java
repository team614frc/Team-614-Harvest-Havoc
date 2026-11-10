// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Ports;
import java.util.function.DoubleSupplier;

/**
 * Elevator driven by one or two Kraken X44s (TalonFX) using Motion Magic.
 *
 * <p>Positions are in rotations (see ELEVATOR_GEAR_RATIO). Zero is the bottom hard stop. The
 * motor's encoder forgets its position on every boot, so the elevator must be homed first; until
 * then setpoints are ignored.
 */
public class Elevator extends SubsystemBase {

  /** The four heights the elevator needs this season. Values live in ElevatorConstants. */
  public enum Setpoint {
    kFeeder(ElevatorConstants.ELEVATOR_FEEDER_SETPOINT),
    kL1(ElevatorConstants.ELEVATOR_L1_SETPOINT),
    kL2(ElevatorConstants.ELEVATOR_L2_SETPOINT),
    kL3(ElevatorConstants.ELEVATOR_L3_SETPOINT);

    public final double position;

    Setpoint(double position) {
      this.position = position;
    }
  }

  // Motors
  private final TalonFX elevatorLeader = new TalonFX(Ports.kElevatorLeader, Ports.kRoboRioCANBus);
  private final TalonFX elevatorFollower =
      ElevatorConstants.ELEVATOR_HAS_FOLLOWER
          ? new TalonFX(Ports.kElevatorFollower, Ports.kRoboRioCANBus)
          : null;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Sensor readings
  private final StatusSignal<Angle> position = elevatorLeader.getPosition();
  private final StatusSignal<AngularVelocity> velocity = elevatorLeader.getVelocity();

  // Control requests (FOC off because it needs a Phoenix Pro license)
  private final MotionMagicVoltage motionMagic =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(false);

  // State
  private double elevatorSetpoint = ElevatorConstants.ELEVATOR_MIN_POSITION;
  private boolean homed = false;

  // Homing helpers
  private final Timer homingTimer = new Timer();
  private Debouncer homingDebouncer = new Debouncer(ElevatorConstants.ELEVATOR_HOMING_STILL_TIME);
  private boolean bottomDetected = false;

  /** Creates a new Elevator. */
  public Elevator() {
    buildConfig();
    applyConfig(elevatorLeader);

    if (elevatorFollower != null) {
      applyConfig(elevatorFollower);
      elevatorFollower.setControl(
          new Follower(
              Ports.kElevatorLeader,
              ElevatorConstants.ELEVATOR_FOLLOWER_OPPOSES_LEADER
                  ? MotorAlignmentValue.Opposed
                  : MotorAlignmentValue.Aligned));
    }

    BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity);
    elevatorLeader.setPosition(0); // meaningless until homed
  }

  // ------------------------------------------------------------------ Configuration

  private void buildConfig() {
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        ElevatorConstants.ELEVATOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Gear ratio: positions/velocities are reported at the mechanism, not the motor.
    config.Feedback.SensorToMechanismRatio = ElevatorConstants.ELEVATOR_GEAR_RATIO;

    config.CurrentLimits.StatorCurrentLimit =
        ElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit =
        ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // PID + feedforward (slot 0)
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kP = ElevatorConstants.kP;
    config.Slot0.kI = ElevatorConstants.kI;
    config.Slot0.kD = ElevatorConstants.kD;
    config.Slot0.kS = ElevatorConstants.kS;
    config.Slot0.kG = ElevatorConstants.kG;
    config.Slot0.kV = ElevatorConstants.kV;
    config.Slot0.kA = ElevatorConstants.kA;

    // Motion Magic profile (jerk 0 = trapezoidal)
    config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.ELEVATOR_MAX_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ELEVATOR_MAX_ACCELERATION;
    config.MotionMagic.MotionMagicJerk = 0;

    // Soft limits stay OFF until homed, because "zero" means nothing before that.
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.ELEVATOR_MAX_POSITION;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.ELEVATOR_MIN_POSITION;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
  }

  private void applyConfig(TalonFX motor) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5 && !status.isOK(); i++) {
      status = motor.getConfigurator().apply(config);
    }
    if (!status.isOK()) {
      DriverStation.reportError(
          "Elevator: failed to configure TalonFX " + motor.getDeviceID() + ": " + status, false);
    }
  }

  private void setSoftLimitsEnabled(boolean enabled) {
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enabled;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enabled;
    elevatorLeader.getConfigurator().apply(config.SoftwareLimitSwitch);
  }

  // ------------------------------------------------------------------ State

  private double getElevatorPosition() {
    return position.getValueAsDouble();
  }

  public boolean isHomed() {
    return homed;
  }

  public boolean reachedSetpoint() {
    return homed
        && Math.abs(getElevatorPosition() - elevatorSetpoint)
            <= ElevatorConstants.ELEVATOR_TOLERANCE;
  }

  // ------------------------------------------------------------------ Control

  private void moveToPosition(double target) {
    if (!homed) {
      DriverStation.reportWarning("Elevator: ignoring setpoint, not homed yet!", false);
      return;
    }
    elevatorSetpoint =
        MathUtil.clamp(
            target,
            ElevatorConstants.ELEVATOR_MIN_POSITION,
            ElevatorConstants.ELEVATOR_MAX_POSITION);
    elevatorLeader.setControl(motionMagic.withPosition(elevatorSetpoint));
  }

  /** Command to set the elevator setpoint. Finishes instantly; the motor keeps holding it. */
  public Command setSetpoint(Setpoint setpoint) {
    return this.runOnce(() -> moveToPosition(setpoint.position))
        .withName("Elevator " + setpoint.name());
  }

  /**
   * Manual control for finding setpoints: stick up/down moves the elevator. When released, it holds
   * wherever it is. Watch "Elevator Actual Position" on the dashboard.
   */
  public Command jog(DoubleSupplier input) {
    return this.runEnd(
            () -> {
              double stick =
                  MathUtil.applyDeadband(
                      input.getAsDouble(), ElevatorConstants.ELEVATOR_JOG_DEADBAND);
              elevatorLeader.setControl(
                  voltageOut.withOutput(
                      ElevatorConstants.kG + stick * ElevatorConstants.ELEVATOR_JOG_MAX_VOLTS));
            },
            () -> {
              if (homed) {
                moveToPosition(getElevatorPosition()); // hold here
              } else {
                elevatorLeader.stopMotor();
              }
            })
        .withName("Elevator Jog");
  }

  /**
   * Slowly drives down until the carriage stops on the bottom hard stop, calls that spot zero, then
   * turns on the soft limits. Hold the button until it finishes.
   */
  public Command homeCommand() {
    return Commands.sequence(
            this.runOnce(
                () -> {
                  homed = false;
                  bottomDetected = false;
                  setSoftLimitsEnabled(false);
                  homingDebouncer = new Debouncer(ElevatorConstants.ELEVATOR_HOMING_STILL_TIME);
                  homingTimer.restart();
                }),
            this.run(
                    () ->
                        elevatorLeader.setControl(
                            voltageOut.withOutput(-ElevatorConstants.ELEVATOR_HOMING_VOLTS)))
                .until(this::stalledAtBottom)
                .withTimeout(ElevatorConstants.ELEVATOR_HOMING_TIMEOUT),
            this.runOnce(this::finishHoming))
        .finallyDo(
            interrupted -> {
              if (interrupted) {
                elevatorLeader.stopMotor();
              }
            })
        .withName("Elevator Home");
  }

  private boolean stalledAtBottom() {
    boolean still =
        Math.abs(velocity.getValueAsDouble()) < ElevatorConstants.ELEVATOR_HOMING_STILL_VELOCITY;
    boolean result =
        homingDebouncer.calculate(still)
            && homingTimer.get() > ElevatorConstants.ELEVATOR_HOMING_MIN_TIME;
    if (result) {
      bottomDetected = true;
    }
    return result;
  }

  private void finishHoming() {
    if (!bottomDetected) {
      elevatorLeader.stopMotor();
      DriverStation.reportError("Elevator: homing timed out, never hit the bottom!", false);
      return;
    }
    elevatorLeader.setPosition(ElevatorConstants.ELEVATOR_MIN_POSITION);
    homed = true;
    setSoftLimitsEnabled(true);
    moveToPosition(ElevatorConstants.ELEVATOR_MIN_POSITION); // hold at the bottom
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(position, velocity);

    SmartDashboard.putBoolean("Elevator Homed", homed);
    SmartDashboard.putNumber("Elevator Target Position", elevatorSetpoint);
    SmartDashboard.putNumber("Elevator Actual Position", getElevatorPosition());
    SmartDashboard.putNumber("Elevator Velocity", velocity.getValueAsDouble());
    SmartDashboard.putBoolean("Elevator At Setpoint", reachedSetpoint());
  }
}
