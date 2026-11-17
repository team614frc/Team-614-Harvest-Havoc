package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ManipulatorPivot;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // Subsystems
  private final Swerve drivebase = new Swerve();
  private final Elevator elevator = new Elevator();
  private final ManipulatorPivot manipulatorPivot = new ManipulatorPivot();
  private final Manipulator manipulator = new Manipulator();

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
    configureBindings();
    configureAutos();
  }

  private void configureBindings() {
    // ---------------------------------------------------------------- Driver
    // The driver only drives, picks up, and scores.

    // Default command: drive with the sticks
    drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));

    // Start = zero gyro
    driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));

    // X held = lock wheels in an X pattern
    driverXbox.x().whileTrue(drivebase.run(drivebase::lockWheels));

    // B held = point-to-direction drive mode (optional, remove if you don't want it)
    driverXbox.b().whileTrue(drivebase.driveFieldOriented(driveDirectAngle));

    // Left trigger held = intake
    driverXbox.leftTrigger().whileTrue(manipulator.intake()).onFalse(manipulator.stop());

    // Right trigger held = outtake
    driverXbox.rightTrigger().whileTrue(manipulator.outtake()).onFalse(manipulator.stop());

    new Trigger(manipulator::hasGamePiece).onTrue(manipulator.stop());

    // LEFT TRIGGER held = pickup: elevator to pickup AND pivot to intake, at the same time
    driverXbox.leftTrigger().whileTrue(pickupCommand());

    // TODO: scoring (roller outtake) goes on the driver's right trigger once the roller is added
    // driverXbox.rightTrigger().whileTrue(manipulator.outtake());

    // ---------------------------------------------------------------- Codriver
    // Elevator: right stick Y moves it (up = up). It is the default command, so the elevator
    // holds its position whenever the stick is centered.
    elevator.setDefaultCommand(elevator.manualControl(() -> -codriverXbox.getRightY()));

    // Pivot: left stick Y moves it (up = toward higher angles). Default command, so the pivot
    // holds whenever the stick is centered.
    manipulatorPivot.setDefaultCommand(
        manipulatorPivot.manualControl(() -> -codriverXbox.getLeftY()));

    // Combined moves (HOLD the button; releasing it cancels both and returns to the sticks)
    // A = elevator to oven AND pivot to oven
    codriverXbox.a().whileTrue(ovenCommand());
    // X / Y / B = pivot to score AND elevator to L1 / L2 / L3
    codriverXbox.x().whileTrue(scoreCommand(elevator.goToL1()));
    codriverXbox.y().whileTrue(scoreCommand(elevator.goToL2()));
    codriverXbox.b().whileTrue(scoreCommand(elevator.goToL3()));

    // D-pad down = home the elevator (lowers until it stalls on the bottom stop, then zeroes)
    codriverXbox.povDown().whileTrue(elevator.homeElevator());

    // D-pad up = home the manipulator pivot (drives toward the reference stop until it stalls,
    // then zeroes). Locks out the pivot stick while it runs.
    codriverXbox.povUp().whileTrue(manipulatorPivot.homeManipulator());

    // Back = declare the current ELEVATOR position as the bottom. Only press this with the
    // carriage resting on the bottom stop (works while disabled too).
    codriverXbox.back().onTrue(elevator.zeroHere());

    // Start = declare the current PIVOT position as the reference stop (MIN_ANGLE) and turn on its
    // stop-side soft limit. Only press this with the pivot resting on its stop (works while
    // disabled too). Do this after every power cycle.
    codriverXbox.start().onTrue(manipulatorPivot.zeroHere());
  }

  // ------------------------------------------------------------------ Combined moves
  // Each one runs the elevator move and the pivot move AT THE SAME TIME. Neither move ends on its
  // own: both keep holding their targets (Motion Magic) until the button is released.

  /** Elevator down to pickup and pivot down to the intake angle. */
  private Command pickupCommand() {
    return Commands.parallel(elevator.goToPickup(), manipulatorPivot.goToIntake())
        .withName("Pickup");
  }

  /** Elevator up to the oven height and pivot to the oven angle. */
  private Command ovenCommand() {
    return Commands.parallel(elevator.goToOven(), manipulatorPivot.goToOven()).withName("Oven");
  }

  /** Pivot to the score angle and the elevator to the given level. */
  private Command scoreCommand(Command elevatorLevel) {
    return Commands.parallel(manipulatorPivot.goToScore(), elevatorLevel).withName("Score");
  }

  // ------------------------------------------------------------------ Autos

  /**
   * Runs an elevator move until the carriage arrives (or the timeout), so it can be chained in an
   * auto. The moves themselves never end, which is what we want while a button is held.
   */
  private Command toSetpoint(Command move) {
    return move.until(elevator::atSetpoint)
        .withTimeout(ElevatorConstants.SETPOINT_TIMEOUT.in(Seconds));
  }

  /** Same as toSetpoint, but for a pivot move. */
  private Command toPivotSetpoint(Command move) {
    return move.until(manipulatorPivot::atSetpoint)
        .withTimeout(PivotConstants.SETPOINT_TIMEOUT.in(Seconds));
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

    autoChooser.addOption("Pivot: Intake", toPivotSetpoint(manipulatorPivot.goToIntake()));
    autoChooser.addOption("Pivot: Oven", toPivotSetpoint(manipulatorPivot.goToOven()));
    autoChooser.addOption("Pivot: Score", toPivotSetpoint(manipulatorPivot.goToScore()));

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
