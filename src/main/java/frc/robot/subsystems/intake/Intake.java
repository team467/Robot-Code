package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.COLLAPSE_VOLTS;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_VOLTS;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_VOLTS;
import static frc.robot.subsystems.intake.IntakeConstants.OUTTAKE_VOLTS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public static Timer stowTimer = new Timer();
  private boolean stalled = true;
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    if (inputs.extendVolts > 0.1 && stalled) {
      stalled = false;
    }
    if (isSlipping()) {
      stowTimer.start();
    } else {
      stowTimer.stop();
      stowTimer.reset();
    }

    if (stowTimer.get() > 0.05) {
      stalled = true;
      stowTimer.stop();
      stowTimer.reset();
    }

    inputs.isStowed = stalled && !isHopperExtended();
  }

  public boolean isStowed() {
    return inputs.isStowed;
  }

  public void setPercentIntake(double intakePercent) {
    io.setPercentIntake(intakePercent);
  }

  public void setPercentExtend(double extendPercent) {
    io.setPercentIntake(extendPercent);
  }

  public void setVoltageIntake(double intakeVolts) {
    io.setVoltageIntake(intakeVolts);
  }

  public void setVoltageExtend(double extendVolts) {
    io.setVoltageIntake(extendVolts);
  }

  public void stopIntake() {
    io.setVoltageIntake(0);
  }

  public void stopExtend() {
    io.setVoltageExtend(0);
  }

  private boolean isHopperExtended() {
    return io.isHopperExtended();
  }

  public Command extend() {
    return Commands.run(
            () -> {
              setVoltageExtend(EXTEND_VOLTS);
            })
        .until(() -> inputs.isExtended)
        .finallyDo(interrupted -> stopExtend());
  }

  public Command intake() {
    return Commands.run(
            () -> {
              setVoltageIntake(INTAKE_VOLTS);
            })
        .finallyDo(interrupted -> stopIntake());
  }

  public Command outtake() {
    return Commands.run(
            () -> {
              setVoltageIntake(OUTTAKE_VOLTS);
            })
        .finallyDo(interrupted -> stopIntake());
  }

  public Command extendAndIntake() {
    return Commands.deadline(extend(), intake()).andThen(intake());
  }

  public Command stopIntakeCommand() {
    return Commands.run(this::stopIntake);
  }

  public Command stopExtendingCommand() {
    return Commands.run(this::stopExtend);
  }

  public Command collapse() {
    return Commands.run(
            () -> {
              setVoltageExtend(COLLAPSE_VOLTS);
            })
        .until(() -> !inputs.isExtended && inputs.isStowed)
        .finallyDo(interrupted -> stopExtend());
  }

  public Command collapseAndIntake() {
    return Commands.deadline(collapse(), intake()).andThen(intake());
  }

  public boolean isSlipping() {
    return Math.abs(inputs.extendVelocity) < 0.1 && inputs.extendVolts < -0.1;
  }
}
