// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.util.Units;
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
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int CODRIVER_CONTROLLER_PORT = 1;
    public static final double DEADBAND = 0.05;
  }

  public static class DrivebaseConstants {
    public static final double MAX_SPEED = Units.feetToMeters(17.5); // 17.5 ft/s
  }

  public static class ElevatorConstants {
    // ---- Wiring / direction
    public static final boolean ELEVATOR_HAS_FOLLOWER = true; // TODO: false if only one motor
    // false = follower spins same way as leader, true = opposite (motors facing each other)
    public static final boolean ELEVATOR_FOLLOWER_OPPOSES_LEADER = true; // TODO
    // Flip if the elevator goes DOWN when the stick is pushed UP
    public static final boolean ELEVATOR_INVERTED = false; // TODO

    // ---- Current limits
    public static final Current ELEVATOR_STATOR_CURRENT_LIMIT = Amps.of(60);
    public static final Current ELEVATOR_SUPPLY_CURRENT_LIMIT = Amps.of(40);

    // ---- Manual control (co-driver stick). These are VOLTS.
    // They can also be changed live on the dashboard, then copied here.
    public static final double ELEVATOR_MANUAL_UP_VOLTS = 2.0; // full stick up (start low!)
    public static final double ELEVATOR_MANUAL_DOWN_VOLTS = 1.0; // full stick down (gravity helps)
    public static final double ELEVATOR_HOLD_VOLTS = 0.0; // TODO: volts that keep it from sagging
    public static final double ELEVATOR_MANUAL_DEADBAND = 0.1;
    public static final double ELEVATOR_MANUAL_RAMP = 12.0; // volts per second (smooths stick)

    // Caps on the live dashboard values, so a typo cannot send full power
    public static final double ELEVATOR_MAX_MANUAL_VOLTS = 5.0;
    public static final double ELEVATOR_MAX_HOLD_VOLTS = 2.0;

    // Motor turns per turn of the output shaft (20:1 gearbox). Positions below are in OUTPUT
    // shaft rotations, not motor rotations.
    public static final double ELEVATOR_GEAR_RATIO = 20.0;

    // ---- Travel limits (motor rotations above the bottom stop)
    public static final double ELEVATOR_MIN_POSITION = 0.0;

    /**
     * Highest SAFE position, a bit below the real top hard stop. PLACEHOLDER: find the real value
     * by driving up slowly and reading "Elevator Actual Position" on the dashboard.
     */
    public static final double ELEVATOR_MAX_POSITION = 10.0; // TODO
  }
}
