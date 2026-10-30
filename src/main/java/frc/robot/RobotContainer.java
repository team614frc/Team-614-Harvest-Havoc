// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Intake intake = new Intake();

  // Controllers
  private final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController codriverXbox =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
  private final CommandXboxController outreachXbox =
      new CommandXboxController(OperatorConstants.OUTREACH_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Home the pivot whenever the robot is enabled in auto or teleop
    RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()).onTrue(intake.homingCommand());

    // Driver
    driverXbox.leftTrigger().whileTrue(intake.intakeCommand());
    driverXbox.y().onTrue(intake.stowCommand());

    // Codriver
    codriverXbox.y().onTrue(intake.stowCommand());
    codriverXbox.leftTrigger().whileTrue(intake.intakeCommandNoPivot());
    codriverXbox.x().onTrue(intake.stopPivotCommand());
    codriverXbox.start().onTrue(intake.homingCommand());
    codriverXbox.povDown().whileTrue(intake.ejectCommand()); // spit game piece out
    codriverXbox.povUp().whileTrue(intake.agitateCommand()); // wiggle pivot to unjam

    // Outreach
    outreachXbox.leftTrigger().whileTrue(intake.intakeCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}