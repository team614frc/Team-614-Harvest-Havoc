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

  public static final class ElevatorConstants {

  // PLACEHOLDERS. Deliberately tiny so nothing can slam into the top before
    // we should replace these later using CTRE tuning. still unsure how to get the real values must research later.
    public static final double ELEVATOR_FEEDER_SETPOINT = 2.0; // TODO
    public static final double ELEVATOR_L1_SETPOINT = 4.0; // TODO
    public static final double ELEVATOR_L2_SETPOINT = 6.0; // TODO
    public static final double ELEVATOR_L3_SETPOINT = 8.0; // TODO
 
    public static final double ELEVATOR_MIN_POSITION = 0.0;
    /** Highest SAFE position, a bit below the real top hard stop. */
    public static final double ELEVATOR_MAX_POSITION = 10.0; // TODO
    public static final double ELEVATOR_TOLERANCE = 0.25;
  }
}
