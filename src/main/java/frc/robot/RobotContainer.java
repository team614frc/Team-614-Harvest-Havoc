package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.lib.BLine.FollowPath;
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

  // The list of autos shown on the dashboard
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

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
    configureBLineEvents(); // register event keys before any path runs
    configureBindings();
    configureAutos();
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

    // Codriver - elevator setpoints (HOLD the button; releasing it cancels and returns to stick)
    codriverXbox.leftBumper().whileTrue(elevator.goToPickup());
    codriverXbox.a().whileTrue(elevator.goToOven());
    codriverXbox.x().whileTrue(elevator.goToL1());
    codriverXbox.y().whileTrue(elevator.goToL2());
    codriverXbox.b().whileTrue(elevator.goToL3());
    codriverXbox.start().whileTrue(elevator.homeElevator());
  }

  /**
   * Runs an elevator move until the carriage arrives (or the timeout), so it can be chained in an
   * auto. The moves themselves never end, which is what we want while a button is held.
   */
  private Command toSetpoint(Command move) {
    return move.until(elevator::atSetpoint)
        .withTimeout(ElevatorConstants.SETPOINT_TIMEOUT.in(Seconds));
  }

  /**
   * Keys you can type into a BLine event trigger (the "Lib Key" field). Each key gets its own
   * command object, so don't reuse these commands in a button binding or the chooser.
   */
  private void configureBLineEvents() {
    FollowPath.registerEventTrigger("elevatorPickup", toSetpoint(elevator.goToPickup()));
    FollowPath.registerEventTrigger("elevatorOven", toSetpoint(elevator.goToOven()));
    FollowPath.registerEventTrigger("elevatorL1", toSetpoint(elevator.goToL1()));
    FollowPath.registerEventTrigger("elevatorL2", toSetpoint(elevator.goToL2()));
    FollowPath.registerEventTrigger("elevatorL3", toSetpoint(elevator.goToL3()));
  }

  // The autos you can pick on the dashboard before enabling Autonomous
  private void configureAutos() {
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Elevator: Pickup (bottom)", toSetpoint(elevator.goToPickup()));
    autoChooser.addOption("Elevator: Oven", toSetpoint(elevator.goToOven()));
    autoChooser.addOption("Elevator: Top", toSetpoint(elevator.goToTop()));
    autoChooser.addOption("Elevator: L1", toSetpoint(elevator.goToL1()));
    autoChooser.addOption("Elevator: L2", toSetpoint(elevator.goToL2()));
    autoChooser.addOption("Elevator: L3", toSetpoint(elevator.goToL3()));

    autoChooser.addOption(
        "Elevator: Pickup then Top",
        Commands.sequence(
            toSetpoint(elevator.goToPickup()),
            Commands.waitSeconds(1.0), // pretend to pick something up
            toSetpoint(elevator.goToTop())));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
