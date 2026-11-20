// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Ports;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Manipulator pivot: one Kraken X44 (TalonFX). The co-driver stick moves it manually, and three
 * setpoint commands (intake, oven, score) move it with CTRE Motion Magic.
 *
 * <p>Angles are measured from HORIZONTAL (0 deg = parallel to the ground, up = positive). That
 * matters because the pivot uses Arm_Cosine gravity compensation, which needs the motor to report 0
 * when the pivot is horizontal. The pivot must START each power-up resting on its reference stop
 * (MIN_ANGLE). At power-on the motor reads 0, so the constructor tells it "you are at MIN_ANGLE".
 * If only the robot code restarts, the motor keeps its real angle and is left alone. If the pivot
 * was NOT on the stop at power-up (or after a brownout), put it on the stop and press the zero
 * button (zeroHere).
 *
 * <p>Unlike the elevator there is no height conversion: with the gear ratio configured, the motor
 * already reports pivot rotations, which are angles.
 */
public class ManipulatorPivot extends SubsystemBase {

  // Dashboard names for the live-adjustable values
  private static final String kRaiseKey = "Pivot Manual Raise Volts";
  private static final String kLowerKey = "Pivot Manual Lower Volts";
  private static final String kHoldKey = "Pivot Hold Volts";
  private static final String kIntakeKey = "Pivot Tuning/Intake (deg)";
  private static final String kOvenKey = "Pivot Tuning/Oven (deg)";
  private static final String kScoreKey = "Pivot Tuning/Score (deg)";
  private static final String kMinKey = "Pivot Tuning/Min Angle (deg)";
  private static final String kMaxKey = "Pivot Tuning/Max Angle (deg)";
  private static final String kVelKey = "Pivot Tuning/Max Speed (deg per s)";
  private static final String kAccelKey = "Pivot Tuning/Max Accel (deg per s2)";

  private double lastMaxRot = Double.NaN;
  private double lastMinRot = Double.NaN;
  private double lastVelRps = Double.NaN;
  private double lastAccelRps2 = Double.NaN;

  // Motors
  private final TalonFX pivotMotor = new TalonFX(Ports.kPivotMotor, Ports.kRoboRioCANBus);

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final StatusSignal<Angle> position = pivotMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = pivotMotor.getVelocity();

  // FOC off because it needs a Phoenix Pro license
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(false);
  private final MotionMagicVoltage motionMagic =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);

  // Smooths the stick so the pivot never gets a sudden jolt
  private final SlewRateLimiter limiter =
      new SlewRateLimiter(PivotConstants.MANUAL_RAMP_VOLTS_PER_SEC);

  // Last Motion Magic target sent (used by atSetpoint() so autos know when a move has arrived)
  private Angle activeTarget = PivotConstants.MIN_ANGLE;

  /** Creates a new ManipulatorPivot. */
  public ManipulatorPivot() {
    buildConfig();
    applyConfig(pivotMotor);

    // Power-on: the motor reads exactly 0, so tell it where it really is (the reference stop).
    // Code-only restart: it keeps its real angle (not near 0), so leave it alone.
    Angle startup = position.waitForUpdate(0.25).getValue();
    if (Math.abs(startup.in(Degrees)) < PivotConstants.POWER_ON_TOLERANCE.in(Degrees)) {
      pivotMotor.setPosition(PivotConstants.MIN_ANGLE);
    }

    // Live-adjustable on the dashboard (clamped where they are read)
    SmartDashboard.putNumber(kRaiseKey, PivotConstants.MANUAL_RAISE.in(Volts));
    SmartDashboard.putNumber(kLowerKey, PivotConstants.MANUAL_LOWER.in(Volts));
    SmartDashboard.putNumber(kHoldKey, PivotConstants.HOLD.in(Volts));
    SmartDashboard.putNumber(kIntakeKey, PivotConstants.INTAKE.in(Degrees));
    SmartDashboard.putNumber(kOvenKey, PivotConstants.OVEN.in(Degrees));
    SmartDashboard.putNumber(kScoreKey, PivotConstants.SCORE.in(Degrees));
    SmartDashboard.putNumber(kMinKey, PivotConstants.MIN_ANGLE.in(Degrees));
    SmartDashboard.putNumber(kMaxKey, PivotConstants.MAX_ANGLE.in(Degrees));
    SmartDashboard.putNumber(kVelKey, PivotConstants.MAX_VELOCITY.in(DegreesPerSecond));
    SmartDashboard.putNumber(kAccelKey, PivotConstants.MAX_ACCEL.in(DegreesPerSecondPerSecond));
  }

  // ------------------------------------------------------------------ Live values (clamped)

  private static double dash(String key, double fallback) {
    return SmartDashboard.getNumber(key, fallback);
  }

  /** Live max angle, capped by the absolute limits. */
  private Angle maxAngle() {
    return Degrees.of(
        MathUtil.clamp(
            dash(kMaxKey, PivotConstants.MAX_ANGLE.in(Degrees)),
            PivotConstants.ABSOLUTE_MIN_ANGLE.in(Degrees),
            PivotConstants.ABSOLUTE_MAX_ANGLE.in(Degrees)));
  }

  /** Live min angle, between the absolute minimum and the live max. */
  private Angle minAngle() {
    return Degrees.of(
        MathUtil.clamp(
            dash(kMinKey, PivotConstants.MIN_ANGLE.in(Degrees)),
            PivotConstants.ABSOLUTE_MIN_ANGLE.in(Degrees),
            maxAngle().in(Degrees)));
  }

  /** Live cruise speed. Floored so a 0 can never be sent (0 means "no limit" on Phoenix). */
  private AngularVelocity maxVelocity() {
    return DegreesPerSecond.of(
        MathUtil.clamp(
            dash(kVelKey, PivotConstants.MAX_VELOCITY.in(DegreesPerSecond)),
            1.0,
            PivotConstants.ABSOLUTE_MAX_VELOCITY.in(DegreesPerSecond)));
  }

  private AngularAcceleration maxAccel() {
    return DegreesPerSecondPerSecond.of(
        MathUtil.clamp(
            dash(kAccelKey, PivotConstants.MAX_ACCEL.in(DegreesPerSecondPerSecond)),
            2.0,
            PivotConstants.ABSOLUTE_MAX_ACCEL.in(DegreesPerSecondPerSecond)));
  }

  private Angle clampAngle(Angle a) {
    return Degrees.of(
        MathUtil.clamp(a.in(Degrees), minAngle().in(Degrees), maxAngle().in(Degrees)));
  }

  /** Pushes changed min/max/speed values to the motor. Does nothing if nothing changed. */
  private void applyLiveLimits() {
    double max = maxAngle().in(Rotations);
    double min = minAngle().in(Rotations);
    double vel = maxVelocity().in(RotationsPerSecond);
    double acc = maxAccel().in(RotationsPerSecondPerSecond);
    if (max == lastMaxRot && min == lastMinRot && vel == lastVelRps && acc == lastAccelRps2) {
      return;
    }
    lastMaxRot = max;
    lastMinRot = min;
    lastVelRps = vel;
    lastAccelRps2 = acc;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min;
    config.MotionMagic.MotionMagicCruiseVelocity = vel;
    config.MotionMagic.MotionMagicAcceleration = acc;

    // config keeps the reverse-limit enable state (it turns on in zeroHere), so this does not
    // switch the bottom limit on early.
    StatusCode a = pivotMotor.getConfigurator().apply(config.SoftwareLimitSwitch);
    StatusCode b = pivotMotor.getConfigurator().apply(config.MotionMagic);
    if (!a.isOK() || !b.isOK()) {
      DriverStation.reportError("Pivot: live limit update failed: " + a + " / " + b, false);
    }
  }

  // ------------------------------------------------------------------ Config

  private void buildConfig() {
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        PivotConstants.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    // Gear ratio: positions are in pivot rotations (angles), not motor rotations.
    config.Feedback.SensorToMechanismRatio = PivotConstants.GEAR_RATIO;

    config.CurrentLimits.StatorCurrentLimit = PivotConstants.STATOR_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = PivotConstants.SUPPLY_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Cap the voltage the motor can ever be told to use, in both directions, whatever is
    // commanding it (stick or Motion Magic). Untuned gains cannot push harder than this.
    config.Voltage.PeakForwardVoltage = PivotConstants.MAX_MANUAL.in(Volts);
    config.Voltage.PeakReverseVoltage = -PivotConstants.MAX_MANUAL.in(Volts);

    // Gains (slot 0), used by Motion Magic. Arm_Cosine: the gravity push is kG at horizontal and
    // shrinks to 0 at straight up/down. This needs the motor to read 0 at horizontal (see the
    // class comment) and one motor rotation per pivot rotation (the gear ratio above).
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kP = PivotConstants.kP;
    config.Slot0.kI = PivotConstants.kI;
    config.Slot0.kD = PivotConstants.kD;
    config.Slot0.kS = PivotConstants.kS;
    config.Slot0.kV = PivotConstants.kV;
    config.Slot0.kA = PivotConstants.kA;
    config.Slot0.kG = PivotConstants.kG;

    // Motion Magic profile (jerk 0 = simple trapezoid: speed up, cruise, slow down).
    // Phoenix wants pivot rot/s and rot/s^2 here.
    config.MotionMagic.MotionMagicCruiseVelocity =
        PivotConstants.MAX_VELOCITY.in(RotationsPerSecond);
    config.MotionMagic.MotionMagicAcceleration =
        PivotConstants.MAX_ACCEL.in(RotationsPerSecondPerSecond);
    config.MotionMagic.MotionMagicJerk = 0;

    // Soft limits. Thresholds are set for both, but only the FAR (max) limit is switched on at
    // boot. The reference-stop (min) limit stays OFF until zeroHere() is pressed, so a wrong zero
    // can never trap the pivot away from its stop.
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PivotConstants.MAX_ANGLE.in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PivotConstants.MIN_ANGLE.in(Rotations);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; // far limit ON
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // stop-side limit OFF until zeroed
  }

  private void applyConfig(TalonFX motor) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5 && !status.isOK(); i++) {
      status = motor.getConfigurator().apply(config);
    }
    if (!status.isOK()) {
      DriverStation.reportError(
          "Pivot: failed to configure TalonFX " + motor.getDeviceID() + ": " + status, false);
    }
  }

  // ------------------------------------------------------------------ Helpers

  /** Reads a dashboard number (in volts), limited to 0..max so a typo cannot send full power. */
  private static Voltage tunable(String key, Voltage fallback, Voltage max) {
    return Volts.of(
        MathUtil.clamp(SmartDashboard.getNumber(key, fallback.in(Volts)), 0, max.in(Volts)));
  }

  private Angle getAngle() {
    return position.getValue();
  }

  /** Volts that hold the pivot where it is: the horizontal hold voltage times cos(angle). */
  private double holdVolts() {
    double atHorizontal = tunable(kHoldKey, PivotConstants.HOLD, PivotConstants.MAX_HOLD).in(Volts);
    return atHorizontal * Math.cos(getAngle().in(Radians));
  }

  // ------------------------------------------------------------------ Manual control

  /**
   * Stick up = toward higher angles, stick down = toward lower angles. Meant to be the default
   * command, so when the stick is centered the pivot holds with the hold voltage (brake mode does
   * the rest).
   */
  public Command manualControl(DoubleSupplier input) {
    return this.runEnd(
            () -> {
              Voltage raise =
                  tunable(kRaiseKey, PivotConstants.MANUAL_RAISE, PivotConstants.MAX_MANUAL);
              Voltage lower =
                  tunable(kLowerKey, PivotConstants.MANUAL_LOWER, PivotConstants.MAX_MANUAL);

              double stick =
                  MathUtil.applyDeadband(input.getAsDouble(), PivotConstants.MANUAL_DEADBAND);
              double volts =
                  holdVolts() + (stick > 0 ? stick * raise.in(Volts) : stick * lower.in(Volts));

              pivotMotor.setControl(voltageOut.withOutput(Volts.of(limiter.calculate(volts))));
            },
            () -> pivotMotor.stopMotor())
        .beforeStarting(() -> limiter.reset(holdVolts()))
        .withName("Pivot Manual");
  }

  /**
   * Declares the current position to be the reference stop (MIN_ANGLE) and turns on the stop-side
   * soft limit. Only press this with the pivot resting on its reference stop. Works while disabled.
   * It does not require the subsystem, so the stick keeps working while it runs.
   */
  public Command zeroHere() {
    return Commands.runOnce(
            () -> {
              pivotMotor.setPosition(PivotConstants.MIN_ANGLE);
              config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
              pivotMotor.getConfigurator().apply(config.SoftwareLimitSwitch);
            })
        .ignoringDisable(true)
        .withName("Pivot Zero Here");
  }

  // ------------------------------------------------------------------ Setpoints (Motion Magic)

  /** Moves to the intake angle (dashboard: Pivot Tuning/Intake). */
  public Command goToIntake() {
    return goToTunable(kIntakeKey, PivotConstants.INTAKE).withName("Pivot Intake");
  }

  /** Moves to the oven angle (dashboard: Pivot Tuning/Oven). */
  public Command goToOven() {
    return goToTunable(kOvenKey, PivotConstants.OVEN).withName("Pivot Oven");
  }

  /** Moves to the score angle (dashboard: Pivot Tuning/Score). */
  public Command goToScore() {
    return goToTunable(kScoreKey, PivotConstants.SCORE).withName("Pivot Score");
  }

  /** Reads the dashboard angle (degrees) every loop, clamped to the live min/max. */
  private Command goToTunable(String key, Angle fallback) {
    return goToPosition(() -> clampAngle(Degrees.of(dash(key, fallback.in(Degrees)))));
  }

  /**
   * Sends the motor a Motion Magic request: the motor controller builds a smooth move (speed up,
   * cruise, slow down) and holds the target. This command NEVER ends on its own, so the motor keeps
   * holding the target (kG + kP) for as long as it is scheduled (for example while a button is
   * held). Releasing the button cancels it and hands back to the stick (manual hold voltage). For
   * autos, add the end condition outside with .until(pivot::atSetpoint), see toPivotSetpoint() in
   * RobotContainer. The soft limits and the peak voltage cap still protect the pivot.
   */
  private Command goToPosition(Supplier<Angle> target) {
    return this.runEnd(
            () -> {
              activeTarget = target.get();
              pivotMotor.setControl(motionMagic.withPosition(activeTarget));
            },
            () -> pivotMotor.setControl(voltageOut.withOutput(Volts.of(holdVolts()))))
        // Set the target before the first check, so atSetpoint() never reads the previous target.
        .beforeStarting(() -> activeTarget = target.get());
  }

  /** True when the pivot is within tolerance of the last Motion Magic target. */
  public boolean atSetpoint() {
    return Math.abs(activeTarget.in(Degrees) - getAngle().in(Degrees))
        <= PivotConstants.TOLERANCE.in(Degrees);
  }

  @Override
  public void periodic() {
    applyLiveLimits();
    SmartDashboard.putNumber("Pivot Angle (deg)", position.refresh().getValue().in(Degrees));
    SmartDashboard.putNumber(
        "Pivot Velocity (deg/s)", velocity.refresh().getValue().in(DegreesPerSecond));
  }
}
