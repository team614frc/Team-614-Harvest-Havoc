package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
  // CAN Buses
  public static final CANBus kRoboRioCANBus = new CANBus("rio");

  // Talon FX IDs
  public static final int kElevatorLeader = 10;
  public static final int kElevatorFollower = 11;
  public static final int kManipulator = 21;
  public static final int kPivotMotor = 31;
}
