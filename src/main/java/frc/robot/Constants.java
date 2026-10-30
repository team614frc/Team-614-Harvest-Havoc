// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

   public static final class OperatorConstants {
    // USB order in the Driver Station. Make sure these match what you see there.
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int OUTREACH_CONTROLLER_PORT = 2;
 
    public static final double DEADBAND = 0.1;
  }
  
 public static final class DrivebaseConstants {
    public static final double MAX_SPEED = Units.feetToMeters(17.5);
  }

   public static class KrakenX44 {
    public static final AngularVelocity kFreeSpeed = RPM.of(7530);
  }

}
