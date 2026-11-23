// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int CODRIVER_CONTROLLER_PORT = 1;
    public static final double DEADBAND = 0.05;
  }

  public static class DrivebaseConstants {
    public static final double MAX_SPEED = Units.feetToMeters(17.5); // 17.5 ft/s
  }

  public static class ElevatorConstants {
    // ---- Wiring / direction
    public static final boolean HAS_FOLLOWER = false; // false only for one-motor bench tests
    // false = follower spins same way as leader, true = opposite (motors facing each other)
    public static final boolean FOLLOWER_OPPOSES_LEADER = true; // verify on blocks
    // Flip if the elevator goes DOWN when the stick is pushed UP
    public static final boolean INVERTED = false; // verify on blocks

    // ---- Mechanism
    // Motor turns per turn of the output shaft (20:1 gearbox). Dimensionless.
    public static final double GEAR_RATIO = 20.0;
    // Radius of the drum the cable/chain wraps on. Height <-> rotations conversion uses this.
    // If the elevator is cascade/continuous rigged, use the EFFECTIVE radius (carriage travel
    // per spool turn / 2*pi).
    public static final Distance SPOOL_RADIUS = Inches.of(0.75); // TODO measure

    // ---- Current limits
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(60);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);

    // ---- Manual control (co-driver stick)
    // They can also be changed live on the dashboard (in volts), then copied here.
    public static final Voltage MANUAL_UP = Volts.of(2.0); // full stick up (start low!)
    public static final Voltage MANUAL_DOWN = Volts.of(1.0); // full stick down (gravity helps)
    public static final Voltage HOLD = Volts.of(0.0); // TODO: volts that keep it from sagging
    public static final double MANUAL_DEADBAND = 0.1; // stick fraction, dimensionless
    // SlewRateLimiter only takes a raw double, so the unit is in the name.
    public static final double MANUAL_RAMP_VOLTS_PER_SEC = 12.0;

    // Caps on the live dashboard values and on the motor's peak voltage, so nothing can ever send
    // full power
    public static final Voltage MAX_MANUAL = Volts.of(5.0);
    public static final Voltage MAX_HOLD = Volts.of(2.0);

    // ---- Travel limits (height above the bottom stop)
    public static final Distance MIN_HEIGHT = Inches.of(0);

    /**
     * Highest SAFE height, a bit below the real top hard stop. The top soft limit uses this.
     * PLACEHOLDER: keep small until measured. Find the real value by driving up slowly and reading
     * "Elevator Height (in)" on the dashboard, then check it with a tape measure.
     */
    public static final Distance MAX_HEIGHT = Inches.of(60); // TODO measure

    // ---- Hard caps for the live dashboard values (Elastic can never go past these)
    public static final Distance ABSOLUTE_MAX_HEIGHT = MAX_HEIGHT; // TODO real hard-stop limit
    public static final LinearVelocity ABSOLUTE_MAX_VELOCITY = InchesPerSecond.of(30);
    public static final LinearAcceleration ABSOLUTE_MAX_ACCEL = InchesPerSecond.per(Second).of(60);

    // ---- Setpoints (keep them below MAX_HEIGHT, they are clamped to it)

    public static final Distance PICKUP = Inches.of(0.0);
    public static final Distance OVEN = Inches.of(0.25); // TODO measure
    public static final Distance L1 = Inches.of(0.5); // TODO measure
    public static final Distance L2 = Inches.of(1.0); // TODO measure
    public static final Distance L3 = Inches.of(1.5); // TODO measure
    // Top sits a little below the top soft limit, so it never pushes into the limit.
    public static final Distance TOP_MARGIN = Inches.of(0.25);
    public static final Distance TOP = MAX_HEIGHT.minus(TOP_MARGIN);

    // SAFETY: the top setpoint is blocked until this is true. Set it to true ONLY after
    // MAX_HEIGHT is the real, measured value (a bit below the top hard stop).
    public static final boolean MAX_HEIGHT_VERIFIED = false; // TODO

    // How close counts as "arrived"
    public static final Distance TOLERANCE = Inches.of(0.25);
    // Give up (and just hold) if it has not arrived by then.
    public static final Time SETPOINT_TIMEOUT = Seconds.of(5);

    // ---- Motion Magic (CTRE motion-profiled position control, runs on the motor)
    // Kraken X44 free speed. Only used for the starting guess for kV below.
    private static final AngularVelocity MOTOR_FREE_SPEED = RPM.of(7530);
    private static final double MAX_OUTPUT_RPS =
        MOTOR_FREE_SPEED.in(RotationsPerSecond) / GEAR_RATIO;

    // Gains. These stay raw doubles because Phoenix wants them in volts per OUTPUT-SHAFT
    // rotation (kP), volts per rot/s (kV), volts per rot/s^2 (kA), and volts (kS, kG).
    // UNTUNED starting values: tune in Phoenix Tuner X, then paste the final numbers here.
    public static final double kP = 6.0; // volts per rotation of error (lower if it shakes)
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.0; // volts to just start moving (raise if it stalls short)
    public static final double kG = HOLD.in(Volts); // volts to hold against gravity
    public static final double kV = 12.0 / MAX_OUTPUT_RPS; // theoretical starting guess
    public static final double kA = 0;

    // Profile limits, in carriage height per second. Deliberately slow to start.
    public static final LinearVelocity MAX_VELOCITY = InchesPerSecond.of(10);
    public static final LinearAcceleration MAX_ACCEL = InchesPerSecond.per(Second).of(20);

    // ---- Homing: lower slowly until it stalls on the bottom stop, then zero.
    // All of these are starting guesses. Tune them from the dashboard first
    // (Elevator Tuning/Homing ...), then copy the final values here.
    public static final Voltage HOMING_VOLTAGE = Volts.of(1.0); // magnitude, applied DOWNWARD
    public static final Voltage HOMING_MAX_VOLTAGE = Volts.of(3.0); // dashboard cap
    public static final Current HOMING_STALL_CURRENT = Amps.of(15); // TODO tune
    public static final LinearVelocity HOMING_MOVING_VELOCITY = InchesPerSecond.of(0.5);
    public static final LinearVelocity HOMING_STALL_VELOCITY = InchesPerSecond.of(0.1);
    public static final Time HOMING_DEBOUNCE = Seconds.of(0.25); // stall must last this long
    public static final Time HOMING_TIMEOUT = Seconds.of(10);
  }

  public static class PivotConstants {
    // ---- Wiring / direction
    // Flip if the pivot moves toward LOWER angles when the stick is pushed positive
    public static final boolean INVERTED = false; // TODO verify on blocks

    // ---- Mechanism
    // Motor turns per turn of the pivot. Dimensionless.
    public static final double GEAR_RATIO = 23.0;

    // ---- Current limits
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);

    // ---- Manual control (co-driver stick)
    // They can also be changed live on the dashboard (in volts), then copied here.
    public static final Voltage MANUAL_RAISE = Volts.of(1.0); // full stick toward higher angles
    public static final Voltage MANUAL_LOWER = Volts.of(1.0); // full stick toward lower angles
    // Volts that hold the pivot at HORIZONTAL (scaled by cos(angle) everywhere else)
    public static final Voltage HOLD = Volts.of(0.0); // TODO
    public static final double MANUAL_DEADBAND = 0.1; // stick fraction, dimensionless
    // SlewRateLimiter only takes a raw double, so the unit is in the name.
    public static final double MANUAL_RAMP_VOLTS_PER_SEC = 12.0;

    // Caps on the live dashboard values and on the motor's peak voltage, so nothing can ever send
    // full power
    public static final Voltage MAX_MANUAL = Volts.of(5.0);
    public static final Voltage MAX_HOLD = Volts.of(2.0);

    // ---- Travel limits
    // ANGLES ARE MEASURED FROM HORIZONTAL: 0 deg = parallel to the ground, up = positive.
    // The pivot must START each power-up resting on its reference stop, which is the LOWEST angle
    // (MIN_ANGLE). Angles increase away from that stop. After every power cycle, press the zero
    // button with the pivot on the stop (the code never resets the position by itself).
    public static final Angle MIN_ANGLE = Degrees.of(0); // TODO angle at the reference stop

    /**
     * Highest SAFE angle, a bit before the far hard stop. PLACEHOLDER: keep small until measured.
     */
    public static final Angle MAX_ANGLE = Degrees.of(30); // TODO measure

    // ---- Hard caps for the live dashboard values (Elastic can never go past these)
    // Tied to the travel limits above until the real hard-stop angles are measured. Do not widen
    // these (or set them to something like 0 / 360) without measuring: a typo on the dashboard
    // could then move the soft limit past the hard stop. If MIN_ANGLE is set below horizontal,
    // this floor follows it automatically.
    public static final Angle ABSOLUTE_MIN_ANGLE = MIN_ANGLE; // TODO real hard-stop limit
    public static final Angle ABSOLUTE_MAX_ANGLE = MAX_ANGLE; // TODO real hard-stop limit
    public static final AngularVelocity ABSOLUTE_MAX_VELOCITY = DegreesPerSecond.of(180);
    public static final AngularAcceleration ABSOLUTE_MAX_ACCEL = DegreesPerSecondPerSecond.of(360);

    // ---- Setpoints (keep them between MIN_ANGLE and MAX_ANGLE, they are clamped to it)
    public static final Angle INTAKE = Degrees.of(5); // TODO measure
    public static final Angle OVEN = Degrees.of(15); // TODO measure
    public static final Angle SCORE = Degrees.of(25); // TODO measure

    // How close counts as "arrived"
    public static final Angle TOLERANCE = Degrees.of(2);
    // Give up (and just hold) if it has not arrived by then.
    public static final Time SETPOINT_TIMEOUT = Seconds.of(5);

    // ---- Motion Magic (CTRE motion-profiled position control, runs on the motor)
    // Kraken X44 free speed. Only used for the starting guess for kV below.
    private static final AngularVelocity MOTOR_FREE_SPEED = RPM.of(7530);
    private static final double MAX_OUTPUT_RPS =
        MOTOR_FREE_SPEED.in(RotationsPerSecond) / GEAR_RATIO;

    // Gains. These stay raw doubles because Phoenix wants them in volts per PIVOT rotation (kP),
    // volts per rot/s (kV), volts per rot/s^2 (kA), and volts (kS, kG).
    // UNTUNED starting values: tune in Phoenix Tuner X, then paste the final numbers here.
    // (A pivot turns far fewer rotations than an elevator spool, so kP is much bigger.)
    public static final double kP = 40.0; // volts per rotation of error (about 0.1 V per degree)
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.0; // volts to just start moving (raise if it stalls short)
    public static final double kG = HOLD.in(Volts); // volts to hold at horizontal (Arm_Cosine)
    public static final double kV = 12.0 / MAX_OUTPUT_RPS; // theoretical starting guess
    public static final double kA = 0;

    // Profile limits. Deliberately slow to start.
    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(60);
    public static final AngularAcceleration MAX_ACCEL = DegreesPerSecondPerSecond.of(120);

    // ---- Homing: drive slowly toward the reference stop until it stalls, then zero.
    // All of these are starting guesses. Tune them from the dashboard first
    // (Pivot Tuning/Homing ...), then copy the final values here.
    // Magnitude, applied toward LOWER angles (toward the stop):
    public static final Voltage HOMING_VOLTAGE = Volts.of(1.0);
    public static final Voltage HOMING_MAX_VOLTAGE = Volts.of(3.0); // dashboard cap
    public static final Current HOMING_STALL_CURRENT = Amps.of(15); // TODO tune
    public static final AngularVelocity HOMING_MOVING_VELOCITY = DegreesPerSecond.of(2);
    public static final AngularVelocity HOMING_STALL_VELOCITY = DegreesPerSecond.of(0.5);
    public static final Time HOMING_DEBOUNCE = Seconds.of(0.25); // stall must last this long
    public static final Time HOMING_TIMEOUT = Seconds.of(10);
  }
}
