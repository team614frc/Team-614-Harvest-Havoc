// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Ports;
import java.util.function.DoubleSupplier;

/**
 * Manually driven elevator: one or two Kraken X44s (TalonFX) controlled by the co-driver stick.
 *
 * <p>Position zero is the bottom stop. The motor reads 0 when it is powered on, so the elevator
 * must START at the bottom after every power cycle. The position is NOT reset when only the robot
 * code restarts, so a code crash or redeploy does not lose it. After a power cycle (or brownout)
 * with the carriage up, lower it onto the bottom stop and press the zero button (zeroHere).
 *
 * <p>Units: the TalonFX works in output-shaft rotations (SensorToMechanismRatio). Everything else
 * in this class and in Constants is a typed height (Distance). The ONLY place height and rotations
 * are converted is the toRotations / toHeight helpers below.
 */
public class Elevator extends SubsystemBase {

  // Dashboard names for the live-adjustable values
  private static final String kUpKey = "Elevator Manual Up Volts";
  private static final String kDownKey = "Elevator Manual Down Volts";
  private static final String kHoldKey = "Elevator Hold Volts";
  private static final String kTestKey = "Elevator Test Target (in)";

  // Carriage travel per output-shaft rotation (the spool circumference)
  private static final double kMetersPerRotation =
      2 * Math.PI * ElevatorConstants.SPOOL_RADIUS.in(Meters);

  // Motors
  private final TalonFX elevatorLeader = new TalonFX(Ports.kElevatorLeader, Ports.kRoboRioCANBus);
  private final TalonFX elevatorFollower =
      ElevatorConstants.HAS_FOLLOWER
          ? new TalonFX(Ports.kElevatorFollower, Ports.kRoboRioCANBus)
          : null;

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final StatusSignal<Angle> position = elevatorLeader.getPosition();
  private final StatusSignal<AngularVelocity> velocity = elevatorLeader.getVelocity();

  // FOC off because it needs a Phoenix Pro license
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(false);
  private final MotionMagicVoltage motionMagic =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);

  // Smooths the stick so the elevator never gets a sudden jolt
  private final SlewRateLimiter limiter =
      new SlewRateLimiter(ElevatorConstants.MANUAL_RAMP_VOLTS_PER_SEC);

  // Last Motion Magic target sent (used by atSetpoint() so autos know when a move has arrived)
  private Distance activeTarget = ElevatorConstants.MIN_HEIGHT;

  /** Creates a new Elevator. */
  public Elevator() {
    buildConfig();
    applyConfig(elevatorLeader);

    if (elevatorFollower != null) {
      applyConfig(elevatorFollower);
      elevatorFollower.setControl(
          new Follower(
              Ports.kElevatorLeader,
              ElevatorConstants.FOLLOWER_OPPOSES_LEADER
                  ? MotorAlignmentValue.Opposed
                  : MotorAlignmentValue.Aligned));
    }

    // Note: position is deliberately NOT reset here (see the class comment).

    // Live-adjustable on the dashboard (clamped where they are read)
    SmartDashboard.putNumber(kUpKey, ElevatorConstants.MANUAL_UP.in(Volts));
    SmartDashboard.putNumber(kDownKey, ElevatorConstants.MANUAL_DOWN.in(Volts));
    SmartDashboard.putNumber(kHoldKey, ElevatorConstants.HOLD.in(Volts));
    SmartDashboard.putNumber(kTestKey, ElevatorConstants.TEST_SETPOINT.in(Inches));
  }

  // ------------------------------------------------------------------ Unit conversion
  // The only place height <-> motor rotations happens.

  private static Angle toRotations(Distance height) {
    return Rotations.of(height.in(Meters) / kMetersPerRotation);
  }

  private static Distance toHeight(Angle rotations) {
    return Meters.of(rotations.in(Rotations) * kMetersPerRotation);
  }

  private static double toRotationsPerSecond(LinearVelocity v) {
    return v.in(MetersPerSecond) / kMetersPerRotation;
  }

  private static double toRotationsPerSecondSquared(LinearAcceleration a) {
    return a.in(MetersPerSecondPerSecond) / kMetersPerRotation;
  }

  private static LinearVelocity toLinearVelocity(AngularVelocity w) {
    return MetersPerSecond.of(w.in(RotationsPerSecond) * kMetersPerRotation);
  }

  private static Distance clampHeight(Distance h) {
    return Meters.of(
        MathUtil.clamp(
            h.in(Meters),
            ElevatorConstants.MIN_HEIGHT.in(Meters),
            ElevatorConstants.MAX_HEIGHT.in(Meters)));
  }

  // ------------------------------------------------------------------ Config

  private void buildConfig() {
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        ElevatorConstants.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    // Gear ratio: positions are in output-shaft rotations, not motor rotations.
    config.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO;

    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Cap the voltage the motor can ever be told to use, in both directions, whatever is
    // commanding it (stick or Motion Magic). Untuned gains cannot push harder than this.
    config.Voltage.PeakForwardVoltage = ElevatorConstants.MAX_MANUAL.in(Volts);
    config.Voltage.PeakReverseVoltage = -ElevatorConstants.MAX_MANUAL.in(Volts);

    // Gains (slot 0), used by Motion Magic. Elevator_Static: gravity pulls with a constant force.
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kP = ElevatorConstants.kP;
    config.Slot0.kI = ElevatorConstants.kI;
    config.Slot0.kD = ElevatorConstants.kD;
    config.Slot0.kS = ElevatorConstants.kS;
    config.Slot0.kV = ElevatorConstants.kV;
    config.Slot0.kA = ElevatorConstants.kA;
    config.Slot0.kG = ElevatorConstants.kG;

    // Motion Magic profile (jerk 0 = simple trapezoid: speed up, cruise, slow down).
    // Phoenix wants output-shaft rot/s and rot/s^2 here.
    config.MotionMagic.MotionMagicCruiseVelocity =
        toRotationsPerSecond(ElevatorConstants.MAX_VELOCITY);
    config.MotionMagic.MotionMagicAcceleration =
        toRotationsPerSecondSquared(ElevatorConstants.MAX_ACCEL);
    config.MotionMagic.MotionMagicJerk = 0;

    // Soft limits. Thresholds are set for both, but only the TOP limit is switched on at boot.
    // The bottom limit stays OFF until zeroHere() is pressed, so a wrong zero can never trap the
    // carriage up high.
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        toRotations(ElevatorConstants.MAX_HEIGHT).in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        toRotations(ElevatorConstants.MIN_HEIGHT).in(Rotations);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; // top limit ON
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // bottom limit OFF until zeroed
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

  // ------------------------------------------------------------------ Helpers

  /** Reads a dashboard number (in volts), limited to 0..max so a typo cannot send full power. */
  private static Voltage tunable(String key, Voltage fallback, Voltage max) {
    return Volts.of(
        MathUtil.clamp(SmartDashboard.getNumber(key, fallback.in(Volts)), 0, max.in(Volts)));
  }

  private Voltage holdVolts() {
    return tunable(kHoldKey, ElevatorConstants.HOLD, ElevatorConstants.MAX_HOLD);
  }

  private Distance getHeight() {
    return toHeight(position.getValue());
  }

  /**
   * Stick up = carriage up, stick down = carriage down. Meant to be the default command, so when
   * the stick is centered the elevator holds with the hold voltage (brake mode does the rest).
   */
  public Command manualControl(DoubleSupplier input) {
    return this.runEnd(
            () -> {
              Voltage up =
                  tunable(kUpKey, ElevatorConstants.MANUAL_UP, ElevatorConstants.MAX_MANUAL);
              Voltage down =
                  tunable(kDownKey, ElevatorConstants.MANUAL_DOWN, ElevatorConstants.MAX_MANUAL);
              Voltage hold = holdVolts();

              double stick =
                  MathUtil.applyDeadband(input.getAsDouble(), ElevatorConstants.MANUAL_DEADBAND);
              double volts =
                  hold.in(Volts) + (stick > 0 ? stick * up.in(Volts) : stick * down.in(Volts));

              elevatorLeader.setControl(voltageOut.withOutput(Volts.of(limiter.calculate(volts))));
            },
            () -> elevatorLeader.stopMotor())
        .beforeStarting(() -> limiter.reset(holdVolts().in(Volts)))
        .withName("Elevator Manual");
  }

  /**
   * Declares the current position to be the bottom (zero) and turns on the bottom soft limit. Only
   * press this with the carriage resting on the bottom stop. Works while disabled. It does not
   * require the subsystem, so the stick keeps working while it runs.
   */
  public Command zeroHere() {
    return Commands.runOnce(
            () -> {
              elevatorLeader.setPosition(toRotations(ElevatorConstants.MIN_HEIGHT));
              config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
              elevatorLeader.getConfigurator().apply(config.SoftwareLimitSwitch);
            })
        .ignoringDisable(true)
        .withName("Elevator Zero Here");
  }

  // ------------------------------------------------------------------ Setpoints (Motion Magic)

  /** Moves down to the bottom to pick something up. */
  public Command goToPickup() {
    return goToPosition(ElevatorConstants.PICKUP).withName("Elevator Pickup");
  }

  /** Moves up to the top. Blocked until the real top height has been measured and entered. */
  public Command goToTop() {
    if (!ElevatorConstants.MAX_HEIGHT_VERIFIED) {
      return Commands.runOnce(
              () ->
                  DriverStation.reportError(
                      "Elevator: top setpoint blocked. Measure the real top, set MAX_HEIGHT,"
                          + " then set MAX_HEIGHT_VERIFIED.",
                      false))
          .withName("Elevator Top (blocked)");
    }
    return goToPosition(ElevatorConstants.TOP).withName("Elevator Top");
  }

  /** Moves to level 1. */
  public Command goToL1() {
    return goToLevel(ElevatorConstants.L1).withName("Elevator L1");
  }

  /** Moves to level 2. */
  public Command goToL2() {
    return goToLevel(ElevatorConstants.L2).withName("Elevator L2");
  }

  /** Moves to level 3. */
  public Command goToL3() {
    return goToLevel(ElevatorConstants.L3).withName("Elevator L3");
  }

  /** Moves to the dashboard "Elevator Test Target (in)" (clamped to the legal travel range). */
  public Command goToTestTarget() {
    return goToPosition(this::testTarget).withName("Elevator Test Target");
  }

  private Distance testTarget() {
    return clampHeight(
        Inches.of(SmartDashboard.getNumber(kTestKey, ElevatorConstants.TEST_SETPOINT.in(Inches))));
  }

  /** Same as goToPosition, but the target is clamped to the legal travel range first. */
  private Command goToLevel(Distance target) {
    return goToPosition(clampHeight(target));
  }

  private Command goToPosition(Distance target) {
    return goToPosition(() -> target);
  }

  /**
   * Sends the motor a Motion Magic request: the motor controller builds a smooth move (speed up,
   * cruise, slow down) and holds the target. This command NEVER ends on its own, so the motor keeps
   * holding the target (kG + kP) for as long as it is scheduled (for example while a button is
   * held). Releasing the button cancels it and hands back to the stick (manual hold voltage). For
   * autos, add the end condition outside with .until(elevator::atSetpoint), see toSetpoint() in
   * RobotContainer. The soft limits and the peak voltage cap still protect the elevator.
   */
  private Command goToPosition(java.util.function.Supplier<Distance> target) {
    return this.runEnd(
        () -> {
          activeTarget = target.get();
          elevatorLeader.setControl(motionMagic.withPosition(toRotations(activeTarget)));
        },
        () -> elevatorLeader.setControl(voltageOut.withOutput(holdVolts())));
  }

  /** True when the carriage is within tolerance of the last Motion Magic target. */
  public boolean atSetpoint() {
    return Math.abs(activeTarget.in(Meters) - getHeight().in(Meters))
        <= ElevatorConstants.TOLERANCE.in(Meters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Elevator Height (in)", toHeight(position.refresh().getValue()).in(Inches));
    SmartDashboard.putNumber(
        "Elevator Velocity (in/s)",
        toLinearVelocity(velocity.refresh().getValue()).in(InchesPerSecond));
  }
}
