// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

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
    public static final int driverXbox = 0;
    public static final int codriverXbox = 1;
    public static final double DEADBAND = 0.05;
  }

  public static class DrivebaseConstants {
    public static final double MAX_SPEED = Units.feetToMeters(17.5); // 17.5 ft/s
  }

  public static class KrakenX44 {
    public static final AngularVelocity kFreeSpeed = RPM.of(7530);
  }

  public static class ElevatorConstants {
    // ---- Wiring / direction
    public static final boolean ELEVATOR_HAS_FOLLOWER = true; // TODO: false if only one motor
    // false = follower spins same way as leader, true = opposite (motors facing each other)
    public static final boolean ELEVATOR_FOLLOWER_OPPOSES_LEADER = false; // TODO
    // Flip if the elevator goes DOWN when told to go UP
    public static final boolean ELEVATOR_INVERTED = false; // TODO

    // ---- Units
    // 1.0 = positions are in MOTOR rotations (works with no measurements at all).
    // Put the real gearbox ratio (e.g. 5.0 for 5:1) to work in spool rotations instead.
    public static final double ELEVATOR_GEAR_RATIO = 1.0; // TODO

    // ---- Current limits
    public static final Current ELEVATOR_STATOR_CURRENT_LIMIT = Amps.of(60);
    public static final Current ELEVATOR_SUPPLY_CURRENT_LIMIT = Amps.of(40);

    // ---- Gains (tune in Phoenix Tuner X, then paste final numbers here)
    // Fastest the mechanism could theoretically spin; only used for the starting guesses below.
    private static final double MAX_MECHANISM_RPS =
        KrakenX44.kFreeSpeed.in(RotationsPerSecond) / ELEVATOR_GEAR_RATIO;

    public static final double kP = 1.0; // volts per rotation of error (start low!)
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.0; // TODO tune: volts to just start moving
    public static final double kG = 0.0; // TODO tune: volts to hold carriage against gravity
    public static final double kV = 12.0 / MAX_MECHANISM_RPS; // theoretical starting guess
    public static final double kA = 0;

    // ---- Motion Magic limits (start slow, speed up later)
    // Too slow: raise the 0.4. Jerky, loud, or tripping the current limit: lower the acceleration.
    public static final double ELEVATOR_MAX_VELOCITY = 0.4 * MAX_MECHANISM_RPS; // rot/s
    public static final double ELEVATOR_MAX_ACCELERATION = 2.0 * ELEVATOR_MAX_VELOCITY; // rot/s^2

    // ---- Manual jog (used to find setpoints)
    public static final double ELEVATOR_JOG_MAX_VOLTS = 3.0;
    public static final double ELEVATOR_JOG_DEADBAND = 0.1;

    // ---- Homing (zeroing on the bottom hard stop)
    public static final double ELEVATOR_HOMING_VOLTS = 1.5; // TODO: must be enough to move down
    public static final double ELEVATOR_HOMING_MIN_TIME = 0.5; // s
    public static final double ELEVATOR_HOMING_STILL_VELOCITY = 0.1; // rot/s
    public static final double ELEVATOR_HOMING_STILL_TIME = 0.25; // s
    public static final double ELEVATOR_HOMING_TIMEOUT = 6.0; // s

    // ---- Setpoints (rotations above the bottom stop)
    // PLACEHOLDERS. Deliberately tiny so nothing can slam into the top before
    // you replace them. Find the real values with the jog procedure.
    public static final double ELEVATOR_FEEDER_SETPOINT = 2.0; // TODO
    public static final double ELEVATOR_L1_SETPOINT = 4.0; // TODO
    public static final double ELEVATOR_L2_SETPOINT = 6.0; // TODO
    public static final double ELEVATOR_L3_SETPOINT = 8.0; // TODO

    public static final double ELEVATOR_MIN_POSITION = 0.0;

    /** Highest SAFE position, a bit below the real top hard stop. Soft limit uses this. */
    public static final double ELEVATOR_MAX_POSITION = 10.0; // TODO

    public static final double ELEVATOR_TOLERANCE = 0.25; // rotations
  }
}
