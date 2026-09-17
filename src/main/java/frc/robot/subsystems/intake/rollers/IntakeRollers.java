package frc.robot.subsystems.intake.rollers;

import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_VOLTS;
import static frc.robot.subsystems.intake.IntakeConstants.OUTTAKE_VOLTS;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends SubsystemBase {
  private final IntakeRollersIO io;
  private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/IntakeRollers", inputs);
    RobotState.getInstance().intaking = inputs.intakeVolts > 0;
  }

  private void setPercentIntake(double intakePercent) {
    io.setPercentIntake(intakePercent);
  }

  public void setVoltageIntake(double intakeVolts) {
    io.setVoltageIntake(intakeVolts);
  }

  private void stopIntake() {
    io.setVoltageIntake(0);
  }

  public Command intake() {
    return Commands.run(() -> setVoltageIntake(INTAKE_VOLTS), this)
        .finallyDo(this::stopIntake)
        .withName("intake");
  }

  public Command outtake() {
    return Commands.run(() -> setVoltageIntake(OUTTAKE_VOLTS), this)
        .finallyDo(this::stopIntake)
        .withName("outtake");
  }

  public Command stopIntakeCommand() {
    return Commands.run(this::stopIntake).withName("stopIntakeCommand");
  }
}
