package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
  // CAN Buses
  public static final CANBus kRoboRioCANBus = new CANBus("rio");

  // Talon FX IDs
  public static final int kIntakePivotLeader = 20; // was kIntakePivot
  public static final int kIntakeRollers = 21;
  public static final int kFloor = 22;
  public static final int kFeeder = 23;
  public static final int kShooterLeft = 24;
  public static final int kShooterMiddle = 25;
  public static final int kShooterRight = 26;
  public static final int kIntakePivotFollower = 27; // new, uses the previously unused ID
  public static final int kHanger = 28;

  // PWM Ports
  public static final int kHoodLeftServo = 8;
  public static final int kHoodRightServo = 9;
}