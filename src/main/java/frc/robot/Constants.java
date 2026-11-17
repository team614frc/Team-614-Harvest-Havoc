// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;

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
    public static final double DEADBAND = 0.05;
  }

  public static class DrivebaseConstants {
    public static final double MAX_SPEED = Units.feetToMeters(17.5); // 17.5 ft/s
  }

  public static class ManipulatorConstants {
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final double STATOR_CURRENT_LIMIT = 40.0; // Amps
    public static final double SUPPLY_CURRENT_LIMIT = 40.0; // Amps
    public static final double INTAKE_SPEED = 0.05;
    public static final double OUTTAKE_SPEED = -0.05;
    public static final double REST_SPEED = 0;
    public static final double MIN_OUTPUT = 0.1; // amps
    public static final double MIN_CURRENT = 20.0; // amps
    public static final double GAME_PIECE_DEBOUNCE_SECONDS = 0.1; // seconds
  }
}
