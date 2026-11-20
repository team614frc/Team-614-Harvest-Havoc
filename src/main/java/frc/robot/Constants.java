package frc.robot;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int CODRIVER_CONTROLLER_PORT = 1;
    public static final double DEADBAND = 0.05;
  }

  public static class DrivebaseConstants {
    public static final double MAX_SPEED = Units.feetToMeters(17.5); // 17.5 ft/s
  }

  public static class PivotConstants {
    // ---- Wiring / direction
    // false = follower spins same way as leader, true = opposite (motors facing each other)
    public static final boolean FOLLOWER_OPPOSES_LEADER = false; // TODO verify on blocks
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
    // (MIN_ANGLE). Angles increase away from that stop.
    public static final Angle MIN_ANGLE = Degrees.of(0); // TODO angle at the reference stop

    /**
     * Highest SAFE angle, a bit before the far hard stop. PLACEHOLDER: keep small until measured.
     */
    public static final Angle MAX_ANGLE = Degrees.of(30); // TODO measure

    // ---- Hard caps for the live dashboard values (Elastic can never go past these)
    public static final Angle ABSOLUTE_MIN_ANGLE = Degrees.of(0); // TODO real hard-stop limit
    public static final Angle ABSOLUTE_MAX_ANGLE = Degrees.of(360); // TODO real hard-stop limit
    public static final AngularVelocity ABSOLUTE_MAX_VELOCITY = DegreesPerSecond.of(180);
    public static final AngularAcceleration ABSOLUTE_MAX_ACCEL = DegreesPerSecondPerSecond.of(360);

    // Used at startup to tell "the motor just powered on" (it reads 0) from "only the robot code
    // restarted" (it keeps its real angle).
    public static final Angle POWER_ON_TOLERANCE = Degrees.of(1);

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
  }
}
