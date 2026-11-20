// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
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
    public static final boolean HAS_FOLLOWER = true; // false only for one-motor bench tests
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
    public static final Distance MAX_HEIGHT = Inches.of(2); // TODO measure

    // ---- Hard caps for the live dashboard values (Elastic can never go past these)
    public static final Distance ABSOLUTE_MAX_HEIGHT = Inches.of(60); // TODO real hard-stop limit
    public static final LinearVelocity ABSOLUTE_MAX_VELOCITY = InchesPerSecond.of(30);
    public static final LinearAcceleration ABSOLUTE_MAX_ACCEL = InchesPerSecond.per(Second).of(60);

    // ---- Setpoints (keep them below MAX_HEIGHT, they are clamped to it)
    public static final Distance PICKUP = MIN_HEIGHT;
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
    public static final Time SETPOINT_TIMEOUT = Seconds.of(15);

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
  }
}
