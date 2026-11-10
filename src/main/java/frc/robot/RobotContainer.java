package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // Subsystems
  private final Swerve drivebase = new Swerve();
  private final Elevator elevator = new Elevator();

  // Controllers
  private final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController codriverXbox =
      new CommandXboxController(OperatorConstants.CODRIVER_CONTROLLER_PORT);

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

    // Codriver - elevator: right stick Y moves it (up = up). It is the default command, so the
    // elevator holds its position whenever the stick is centered.
    elevator.setDefaultCommand(elevator.manualControl(() -> -codriverXbox.getRightY()));

    // Back = declare the current position as the bottom. Only press this with the carriage
    // resting on the bottom stop (works while disabled too).
    codriverXbox.back().onTrue(elevator.zeroHere());
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