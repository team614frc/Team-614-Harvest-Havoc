package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
  // CAN Buses
  public static final CANBus kRoboRioCANBus = new CANBus("rio");

  // Talon FX IDs
  public static final int kIntakePivotLeader = 20; // was kIntakePivot
  public static final int kIntakeRollers = 21;
  public static final int kIntakePivotFollower = 27; // new, uses the previously unused ID


}