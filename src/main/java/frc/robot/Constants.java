// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

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
    public static final boolean ELEVATOR_HAS_FOLLOWER =
        true; // false only for one-motor bench tests
    // false = follower spins same way as leader, true = opposite (motors facing each other)
    public static final boolean ELEVATOR_FOLLOWER_OPPOSES_LEADER = true; // verify on blocks
    // Flip if the elevator goes DOWN when the stick is pushed UP
    public static final boolean ELEVATOR_INVERTED = true; // verify on blocks

    // ---- Units
    // Motor turns per turn of the output shaft (20:1 gearbox). All positions and speeds below
    // are in OUTPUT shaft rotations, not motor rotations.
    public static final double ELEVATOR_GEAR_RATIO = 20.0;

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

    // Caps on the live dashboard values and on the motor's peak voltage, so nothing can ever send
    // full power
    public static final double ELEVATOR_MAX_MANUAL_VOLTS = 5.0;
    public static final double ELEVATOR_MAX_HOLD_VOLTS = 2.0;

    // ---- Travel limits (output shaft rotations above the bottom stop)
    public static final double ELEVATOR_MIN_POSITION = 0.0;

    /**
     * Highest SAFE position, a bit below the real top hard stop. The top soft limit uses this.
     * PLACEHOLDER: find the real value by driving up slowly and reading "Elevator Actual Position"
     * on the dashboard.
     */
    public static final double ELEVATOR_MAX_POSITION = 10.0; // TODO

    // ---- Setpoints (used by the auto commands)
    // Bottom = where the elevator picks something up.
    public static final double ELEVATOR_PICKUP_SETPOINT = ELEVATOR_MIN_POSITION;
    // Top sits a little below the top soft limit, so it never pushes into the limit.
    public static final double ELEVATOR_TOP_MARGIN = 0.05;
    public static final double ELEVATOR_TOP_SETPOINT = ELEVATOR_MAX_POSITION - ELEVATOR_TOP_MARGIN;

    // SAFETY: the top setpoint is blocked until this is true. Set it to true ONLY after
    // ELEVATOR_MAX_POSITION is the real, measured value (a bit below the top hard stop).
    public static final boolean ELEVATOR_MAX_POSITION_VERIFIED = true; // TODO

    // How close counts as "arrived" (output shaft rotations)
    public static final double ELEVATOR_TOLERANCE = 0.03;
    // Give up (and just hold) if it has not arrived by then.
    public static final double ELEVATOR_SETPOINT_TIMEOUT = 15.0; // s

    // ---- Motion Magic (CTRE motion-profiled position control, runs on the motor)
    // Kraken X44 free speed (7530 RPM = 125.5 rot/s). Only used for the starting guesses below.
    private static final double MOTOR_FREE_SPEED_RPS = 125.5;
    private static final double MAX_OUTPUT_RPS = MOTOR_FREE_SPEED_RPS / ELEVATOR_GEAR_RATIO;

    // Gains. UNTUNED starting values: tune in Phoenix Tuner X, then paste the final numbers here.
    public static final double kP = 6.0; // volts per rotation of error (lower if it shakes)
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.0; // volts to just start moving (raise if it stalls short)
    public static final double kG = ELEVATOR_HOLD_VOLTS; // volts to hold against gravity
    public static final double kV = 12.0 / MAX_OUTPUT_RPS; // theoretical starting guess
    public static final double kA = 0;

    // Profile limits. Deliberately slow to start: about 15% of top speed.
    public static final double ELEVATOR_MAX_VELOCITY = 0.5 * MAX_OUTPUT_RPS; // rot/s
    public static final double ELEVATOR_MAX_ACCELERATION = .10 * ELEVATOR_MAX_VELOCITY; // rot/s^2

    public static final double ELEVATOR_L1_SETPOINT = 2.0; // TODO
    public static final double ELEVATOR_L2_SETPOINT = 5.0; // TODO
    public static final double ELEVATOR_L3_SETPOINT = 8.0; // TODO  }
  }
}
