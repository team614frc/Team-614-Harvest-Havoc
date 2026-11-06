package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
<<<<<<< HEAD
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveInputStream;
=======
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
>>>>>>> origin/main

public class RobotContainer {
<<<<<<< HEAD
=======
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
>>>>>>> origin/main

  private final Swerve drivebase = new Swerve();

  private final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.driverXbox);

  /** Field-relative drive, rotation controlled by the right stick. */
  private final SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX())
          .withControllerRotationAxis(() -> -driverXbox.getRightX())
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** Right stick sets the direction the robot faces instead of the turn rate. */
  private final SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
          .headingWhile(true);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Default command: drive with the sticks
    drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));

    // Start = zero gyro
    driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));

    // X held = lock wheels in an X pattern
    driverXbox.x().whileTrue(drivebase.run(drivebase::lockWheels));

    // B held = point-to-direction drive mode (optional, remove if you don't want it)
    driverXbox.b().whileTrue(drivebase.driveFieldOriented(driveDirectAngle));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}