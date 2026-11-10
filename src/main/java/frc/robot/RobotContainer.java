package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private static final double MANIPULATOR_SPEED_DEG_PER_SEC = 3.0;

  private final Swerve drivebase = new Swerve();
  private final ManipulatorSubsystem manipulator = new ManipulatorSubsystem();

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

    // Arm
    // Right stick Y = manual voltage control (up raises the arm). Holds position on release.
    manipulator.setDefaultCommand(
        manipulator.manual(
            () -> MathUtil.applyDeadband(-codriverXbox.getRightY(), OperatorConstants.DEADBAND)));

    // ---------------- Codriver: manipulator ----------------
    // B held = manipulator up, A held = manipulator down. Manipulator holds its position when the
    // button is released.
    codriverXbox.b().whileTrue(manipulator.nudge(MANIPULATOR_SPEED_DEG_PER_SEC));
    codriverXbox.a().whileTrue(manipulator.nudge(-MANIPULATOR_SPEED_DEG_PER_SEC));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
