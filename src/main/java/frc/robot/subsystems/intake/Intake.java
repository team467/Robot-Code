package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.COLLAPSE_VOLTS;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_VOLTS;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_VOLTS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  private void setPercent(double intakePercent, double extendPercent) {
    io.setPercent(intakePercent, extendPercent);
  }

  private void setVoltage(double intakeVolts, double extendVolts) {
    io.setVoltage(intakeVolts, extendVolts);
  }

  private void stop() {
    io.setVoltage(0, 0);
  }

  private boolean isHopperExtended() {
    return io.isHopperExtended();
  }

  public Command extend() {
    return Commands.run(
        () -> {
          if (!isHopperExtended()) {
            setVoltage(0, EXTEND_VOLTS);
          }
        });
  }

  public Command intake() {
    return Commands.run(
        () -> {
          if (isHopperExtended()) {
            setVoltage(INTAKE_VOLTS, 0);
          }
        });
  }

  public Command extendAndIntake() {
    return Commands.run(
        () -> {
          if (!isHopperExtended()) {
            setVoltage(INTAKE_VOLTS, EXTEND_VOLTS);
          }
        });
  }

  public Command stopCommand() {
    return Commands.run(
        () -> {
          stop();
        });
  }

  public Command collapse() {
    return Commands.run(
        () -> {
          setVoltage(0, COLLAPSE_VOLTS);
        });
  }
}
