package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Ports;

public class Manipulator extends SubsystemBase {

  // Dashboard names for the live-adjustable values (volts, always entered as positive numbers)
  private static final String kIntakeKey = "Manipulator/Intake Volts";
  private static final String kOuttakeKey = "Manipulator/Outtake Volts";

  // Instantiate the motor controller for the manipulator using the TalonFX class and the ID from
  // the Ports class
  private final TalonFX manipulatorMotor = new TalonFX(Ports.kManipulator);

  // Reusable control request (avoids allocating a new object every loop).
  // FOC off because it needs a Phoenix Pro license.
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);

  // Requires the game piece reading to stay true briefly before we believe it.
  // kRising means only the false -> true change is delayed; going back to false is instant.
  private final Debouncer gamePieceDebouncer =
      new Debouncer(
          ManipulatorConstants.GAME_PIECE_DEBOUNCE.in(Seconds), Debouncer.DebounceType.kRising);

  // Latest debounced result, updated once per loop in periodic()
  private boolean hasGamePiece = false;

  // Create the manipulator and configure its motor
  public Manipulator() {
    configureMotor();

    // Live-adjustable on the dashboard (clamped where they are read)
    SmartDashboard.putNumber(kIntakeKey, ManipulatorConstants.INTAKE_VOLTAGE.in(Volts));
    SmartDashboard.putNumber(kOuttakeKey, ManipulatorConstants.OUTTAKE_VOLTAGE.in(Volts));
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor behavior
    config.MotorOutput.NeutralMode =
        NeutralModeValue.Brake; // resist spinning when no power is applied
    config.MotorOutput.Inverted =
        ManipulatorConstants.MOTOR_INVERTED; // physical direction of the motor

    // Current limits (protects the motor and helps with game piece detection tuning)
    config.CurrentLimits.StatorCurrentLimit = ManipulatorConstants.STATOR_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ManipulatorConstants.SUPPLY_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Cap the voltage the motor can ever be told to use, whatever is commanding it
    config.Voltage.PeakForwardVoltage = ManipulatorConstants.MAX_VOLTAGE.in(Volts);
    config.Voltage.PeakReverseVoltage = -ManipulatorConstants.MAX_VOLTAGE.in(Volts);

    manipulatorMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();

    // Update the debounced game piece reading from the raw current check
    hasGamePiece = gamePieceDebouncer.calculate(isDrawingGamePieceCurrent());

    SmartDashboard.putNumber("Manipulator Current (A)", getManipulatorCurrent().in(Amps));
    SmartDashboard.putBoolean("Manipulator Has Game Piece", hasGamePiece());
  }

  // Exposes the manipulator motor to other classes
  public TalonFX getMotor() {
    return manipulatorMotor;
  }

  /**
   * Reads a dashboard number (in volts), limited to 0..MAX_VOLTAGE so a typo cannot send full
   * power.
   */
  private static Voltage tunable(String key, Voltage fallback) {
    return Volts.of(
        MathUtil.clamp(
            SmartDashboard.getNumber(key, fallback.in(Volts)),
            0,
            ManipulatorConstants.MAX_VOLTAGE.in(Volts)));
  }

  // Set the voltage (positive = intake direction, negative = outtake) for the manipulator motor
  public void set(Voltage volts) {
    manipulatorMotor.setControl(voltageRequest.withOutput(volts));
  }

  // Command to run the manipulator inward (e.g., for gripping or intaking)
  public Command intake() {
    return this.runOnce(() -> set(tunable(kIntakeKey, ManipulatorConstants.INTAKE_VOLTAGE)));
  }

  // Command to run the manipulator outward. The dashboard value is a positive magnitude.
  public Command outtake() {
    return this.runOnce(
        () -> set(tunable(kOuttakeKey, ManipulatorConstants.OUTTAKE_VOLTAGE).unaryMinus()));
  }

  // Command to stop / idle the manipulator
  public Command stop() {
    return this.runOnce(() -> set(ManipulatorConstants.REST_VOLTAGE));
  }

  // Returns true if a game piece is detected (debounced, so the current must stay high briefly)
  public boolean hasGamePiece() {
    return hasGamePiece;
  }

  // Raw check: the motor is being driven forward (intaking) and drawing high current,
  // which means a game piece is squeezing the rollers.
  private boolean isDrawingGamePieceCurrent() {
    Current current = getManipulatorCurrent();
    Voltage output = manipulatorMotor.getMotorVoltage().getValue();

    return output.gt(ManipulatorConstants.MIN_OUTPUT)
        && current.gt(ManipulatorConstants.MIN_CURRENT);
  }

  // Stator (motor) current
  private Current getManipulatorCurrent() {
    return manipulatorMotor.getStatorCurrent().getValue();
  }
}
