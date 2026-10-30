package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
  // CAN Buses
  public static final CANBus kRoboRioCANBus = new CANBus("rio");

  // Talon FX IDs
  public static final int kIntakePivotLeader = 9; // was kIntakePivot
  public static final int kIntakeRollers = 10;
  public static final int kIntakePivotFollower = 11; // new, uses the previously unused ID

 public static final int kElevatorLeader = 12;   
  public static final int kElevatorFollower = 13; 
}