// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
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
    public static final double DEADBAND = 0.05;
  }

  public static class DrivebaseConstants {
    public static final double MAX_SPEED = Units.feetToMeters(17.5); // 17.5 ft/s
  }

  public static class ManipulatorConstants {
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);

    // Roller voltages. These are the starting values; both can be changed live on the dashboard
    // (Manipulator/Intake Volts, Manipulator/Outtake Volts), then copied here.
    // Both are positive magnitudes: the subsystem applies the sign (outtake runs negative).
    // (0.6 V is roughly the old 5% duty cycle on a 12 V battery.)
    public static final Voltage INTAKE_VOLTAGE = Volts.of(0.6);
    public static final Voltage OUTTAKE_VOLTAGE = Volts.of(0.6);
    public static final Voltage REST_VOLTAGE = Volts.of(0);

    // Caps on the live dashboard values and on the motor's peak voltage, so nothing can ever send
    // full power
    public static final Voltage MAX_VOLTAGE = Volts.of(6.0);

    // Game piece detection: the motor must be driven at least this hard in the intake direction...
    // (keep this BELOW INTAKE_VOLTAGE or detection can never trigger)
    public static final Voltage MIN_OUTPUT = Volts.of(0.3);
    // ...while drawing at least this much stator current.
    public static final Current MIN_CURRENT = Amps.of(20);
    public static final Time GAME_PIECE_DEBOUNCE = Seconds.of(0.1);
  }
}
