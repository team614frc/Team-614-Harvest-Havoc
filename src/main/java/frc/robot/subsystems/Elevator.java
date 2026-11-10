// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Ports;
import java.util.function.DoubleSupplier;

/**
 * Manually driven elevator: one or two Kraken X44s (TalonFX) controlled by the co-driver stick.
 *
 * <p>The elevator must START at the bottom stop when the code starts, because that is where
 * position zero is assumed to be. If the code restarts anywhere else, lower the carriage onto the
 * bottom stop and press the zero button (zeroHere).
 */
public class Elevator extends SubsystemBase {

  // Dashboard names for the live-adjustable manual values
  private static final String kUpKey = "Elevator Manual Up Volts";
  private static final String kDownKey = "Elevator Manual Down Volts";
  private static final String kHoldKey = "Elevator Hold Volts";

  // Motors
  private final TalonFX elevatorLeader = new TalonFX(Ports.kElevatorLeader, Ports.kRoboRioCANBus);
  private final TalonFX elevatorFollower =
      ElevatorConstants.ELEVATOR_HAS_FOLLOWER
          ? new TalonFX(Ports.kElevatorFollower, Ports.kRoboRioCANBus)
          : null;

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final StatusSignal<Angle> position = elevatorLeader.getPosition();

  // FOC off because it needs a Phoenix Pro license
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(false);

  // Smooths the stick so the elevator never gets a sudden jolt
  private final SlewRateLimiter limiter =
      new SlewRateLimiter(ElevatorConstants.ELEVATOR_MANUAL_RAMP);

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

    // Assume the elevator starts at the bottom, so this is zero.
    elevatorLeader.setPosition(ElevatorConstants.ELEVATOR_MIN_POSITION);

    // Live-adjustable on the dashboard (clamped in manualControl)
    SmartDashboard.putNumber(kUpKey, ElevatorConstants.ELEVATOR_MANUAL_UP_VOLTS);
    SmartDashboard.putNumber(kDownKey, ElevatorConstants.ELEVATOR_MANUAL_DOWN_VOLTS);
    SmartDashboard.putNumber(kHoldKey, ElevatorConstants.ELEVATOR_HOLD_VOLTS);
  }

  private void buildConfig() {
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        ElevatorConstants.ELEVATOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.StatorCurrentLimit =
        ElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit =
        ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Top soft limit is ON from boot (we assume we start at the bottom), so the stick cannot drive
    // the carriage into the top. The bottom limit stays off until zeroHere() is pressed, so a
    // wrong zero can never trap the carriage up high.
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.ELEVATOR_MAX_POSITION;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.ELEVATOR_MIN_POSITION;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
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

  /** Reads a dashboard number, limited to 0..max so a typo cannot send full power. */
  private static double tunable(String key, double fallback, double max) {
    return MathUtil.clamp(SmartDashboard.getNumber(key, fallback), 0, max);
  }

  /**
   * Stick up = carriage up, stick down = carriage down. Meant to be the default command, so when
   * the stick is centered the elevator holds with the hold voltage (brake mode does the rest).
   */
  public Command manualControl(DoubleSupplier input) {
    return this.runEnd(
            () -> {
              double up =
                  tunable(
                      kUpKey,
                      ElevatorConstants.ELEVATOR_MANUAL_UP_VOLTS,
                      ElevatorConstants.ELEVATOR_MAX_MANUAL_VOLTS);
              double down =
                  tunable(
                      kDownKey,
                      ElevatorConstants.ELEVATOR_MANUAL_DOWN_VOLTS,
                      ElevatorConstants.ELEVATOR_MAX_MANUAL_VOLTS);
              double hold =
                  tunable(
                      kHoldKey,
                      ElevatorConstants.ELEVATOR_HOLD_VOLTS,
                      ElevatorConstants.ELEVATOR_MAX_HOLD_VOLTS);

              double stick =
                  MathUtil.applyDeadband(
                      input.getAsDouble(), ElevatorConstants.ELEVATOR_MANUAL_DEADBAND);
              double volts = hold + (stick > 0 ? stick * up : stick * down);

              elevatorLeader.setControl(voltageOut.withOutput(limiter.calculate(volts)));
            },
            () -> elevatorLeader.stopMotor())
        .beforeStarting(() -> limiter.reset(ElevatorConstants.ELEVATOR_HOLD_VOLTS))
        .withName("Elevator Manual");
  }

  /**
   * Declares the current position to be the bottom (zero) and turns on the bottom soft limit. Only
   * press this with the carriage resting on the bottom stop. Works while disabled.
   */
  public Command zeroHere() {
    return this.runOnce(
            () -> {
              elevatorLeader.setPosition(ElevatorConstants.ELEVATOR_MIN_POSITION);
              config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
              elevatorLeader.getConfigurator().apply(config.SoftwareLimitSwitch);
            })
        .ignoringDisable(true)
        .withName("Elevator Zero Here");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Actual Position", position.refresh().getValueAsDouble());
  }
}