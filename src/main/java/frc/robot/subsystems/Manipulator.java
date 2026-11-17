package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Ports;

public class Manipulator extends SubsystemBase {

  // Instantiate the motor controller for the manipulator using the TalonFX class and the ID from
  // the Ports class
  private final TalonFX manipulatorMotor = new TalonFX(Ports.kManipulator);

  // Reusable control request (avoids allocating a new object every loop)
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

  // Requires the game piece reading to stay true briefly before we believe it.
  // kRising means only the false -> true change is delayed; going back to false is instant.
  private final Debouncer gamePieceDebouncer =
      new Debouncer(
          ManipulatorConstants.GAME_PIECE_DEBOUNCE_SECONDS, Debouncer.DebounceType.kRising);

  // Latest debounced result, updated once per loop in periodic()
  private boolean hasGamePiece = false;

  // Create the manipulator and configure its motor
  public Manipulator() {
    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor behavior
    config.MotorOutput.NeutralMode =
        NeutralModeValue.Brake; // resist spinning when no power is applied
    config.MotorOutput.Inverted =
        ManipulatorConstants.MOTOR_INVERTED; // physical direction of the motor

    // Current limits (protects the motor and helps with game piece detection tuning)
    config.CurrentLimits.StatorCurrentLimit = ManipulatorConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ManipulatorConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    manipulatorMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();

    // Update the debounced game piece reading from the raw current check
    hasGamePiece = gamePieceDebouncer.calculate(isDrawingGamePieceCurrent());

    SmartDashboard.putNumber("Manipulator Current", getManipulatorCurrent());
    SmartDashboard.putBoolean("Manipulator Has Game Piece", hasGamePiece());
  }

  // Exposes the manipulator motor to other classes
  public TalonFX getMotor() {
    return manipulatorMotor;
  }

  // Set the power level (-1.0 to 1.0) for the manipulator motor
  public void set(double speed) {
    manipulatorMotor.setControl(dutyCycleRequest.withOutput(speed));
  }

  // Command to run the manipulator inward (e.g., for gripping or intaking)
  public Command intake() {
    return this.runOnce(() -> set(ManipulatorConstants.INTAKE_SPEED));
  }

  public Command outtake() {
    return this.runOnce(() -> set(ManipulatorConstants.OUTTAKE_SPEED));
  }

  // Command to stop / idle the manipulator
  public Command stop() {
    return this.runOnce(() -> set(ManipulatorConstants.REST_SPEED));
  }

  // Returns true if a game piece is detected (debounced, so the current must stay high briefly)
  public boolean hasGamePiece() {
    return hasGamePiece;
  }

  // Raw check: the motor is being driven forward (intaking) and drawing high current,
  // which means a game piece is squeezing the rollers.
  private boolean isDrawingGamePieceCurrent() {
    double current = getManipulatorCurrent();
    double output = manipulatorMotor.getDutyCycle().getValueAsDouble();

    return (output > ManipulatorConstants.MIN_OUTPUT)
        && (current > ManipulatorConstants.MIN_CURRENT);
  }

  // Stator (motor) current in amps
  private double getManipulatorCurrent() {
    return manipulatorMotor.getStatorCurrent().getValueAsDouble();
  }
}
