// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

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

  public static class KrakenX44 {
    public static final AngularVelocity kFreeSpeed = RPM.of(7530);
  }

  public final class ManipulatorConstants {
  private ManipulatorConstants() {}

 
  // Mechanical
  public static final double GEAR_RATIO = 60.0;          // motor rotations per manipulator rotation
  public static final double MANIPULATOR_LENGTH_M = 0.60;
  public static final double MANIPULATOR_MASS_KG = 4.0;
  public static final double MIN_ANGLE_DEG = -20.0;
  public static final double MAX_ANGLE_DEG = 75.0;
  public static final double STARTING_ANGLE_DEG = -20.0; // where the manipulator rests at boot
  public static final boolean INVERT_LEADER = false;
  public static final boolean FOLLOWER_OPPOSES_LEADER = true; // true if motors face each other
 
  // Current limits (X44 is small - keep these conservative)
  public static final double STATOR_LIMIT_A = 60.0;
  public static final double SUPPLY_LIMIT_A = 40.0;
 
  // Closed loop (voltage based, units: volts per mechanism rotation / rps)
  public static final double kP = 60.0;
  public static final double kI = 0.0;
  public static final double kD = 2.0;
  public static final double kS = 0.15;   // volts to overcome friction
  public static final double kV = 8.0;    // volts per (mechanism rot/sec)
  public static final double kA = 0.1;    // volts per (mechanism rot/sec^2)
  public static final double kG = 0.35;   // volts to hold manipulator horizontal
 
  // Motion Magic profile (mechanism units)
  public static final double CRUISE_VEL_RPS = 0.5;       // mechanism rot/sec
  public static final double ACCEL_RPS2 = 1.5;           // mechanism rot/sec^2
  public static final double JERK_RPS3 = 0.0;            // 0 = trapezoidal
 
  public static final double TOLERANCE_DEG = 2.0;
  }
}