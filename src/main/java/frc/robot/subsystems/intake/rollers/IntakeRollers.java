package frc.robot.subsystems.intake.rollers;

import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_VOLTS;
import static frc.robot.subsystems.intake.IntakeConstants.OUTTAKE_VOLTS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends SubsystemBase {
  private final IntakeRollersIO io;
  private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;
  }

  /** Updates roller telemetry and exposes the active intake state to robot-wide logic. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/IntakeRollers", inputs);
    RobotState.getInstance().intaking = inputs.intakeVolts > 0;
  }

  /** Drives the intake rollers with open-loop voltage. */
  public void setVoltageIntake(double intakeVolts) {
    io.setVoltageIntake(intakeVolts);
  }

  /** Stops the intake rollers immediately. */
  private void stopIntake() {
    io.setVoltageIntake(0);
  }

  /** Runs the rollers inward until the command ends. */
  public Command intake() {
    return Commands.run(() -> setVoltageIntake(INTAKE_VOLTS), this)
        .finallyDo(this::stopIntake)
        .withName("intake");
  }

  /** Runs the rollers outward until the command ends. */
  public Command outtake() {
    return Commands.run(() -> setVoltageIntake(OUTTAKE_VOLTS), this)
        .finallyDo(this::stopIntake)
        .withName("outtake");
  }

  /** Creates a command that keeps the rollers stopped while scheduled. */
  public Command stopIntakeCommand() {
    return Commands.run(this::stopIntake).withName("stopIntakeCommand");
  }
}
