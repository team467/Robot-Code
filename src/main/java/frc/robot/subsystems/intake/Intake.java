package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public static Timer stallTimer = new Timer();
  private boolean stalledCollapse = true;
  private boolean stalledExtend = false;
  private boolean isStowed = false;
  private boolean isExtended = false;
  private double extendPos = 0;
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private BooleanSupplier limitSwitchDisabled;

  public Intake(IntakeIO io, BooleanSupplier limitSwitchDisabled) {
    this.io = io;
    this.limitSwitchDisabled = limitSwitchDisabled;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Calibrate the extend motor
    if (!limitSwitchDisabled.getAsBoolean() && isHopperCollapsed()) {
      io.resetExtendEncoder(0);
    }
    Logger.recordOutput("Intake/stallTimer", stallTimer.get());

    io.extendToPosition(extendPos);

    if (limitSwitchDisabled.getAsBoolean()) {
      if (inputs.extendVolts > 0.05 && stalledExtend) {
        stalledExtend = false;
        isExtended = false;
      }
      if (inputs.extendVolts < -0.05 && stalledCollapse) {
        stalledCollapse = false;
        isStowed = false;
      }

      if (isStallingCollapse()) {
        stallTimer.start();
      } else {
        stallTimer.stop();
        stallTimer.reset();
      }

      if (stallTimer.get() > 0.01 && isStallingCollapse()) {
        stalledCollapse = true;
        stallTimer.stop();
        stallTimer.reset();
      }
      if (stallTimer.get() > 0.01 && isStallingExtend()) {
        stalledCollapse = true;
        stallTimer.stop();
        stallTimer.reset();
      }

      if (stalledCollapse) {
        io.resetExtendEncoder(0);
        isStowed = true;
      }
      if (stalledExtend) {
        io.resetExtendEncoder(EXTEND_POS);
        isExtended = true;
      }
    }
  }

  private void setPercentIntake(double intakePercent) {
    io.setPercentIntake(intakePercent);
  }

  private void setPercentExtend(double extendPercent) {
    io.setPercentIntake(extendPercent);
  }

  private void setVoltageIntake(double intakeVolts) {
    io.setVoltageIntake(intakeVolts);
  }

  private void setVoltageExtend(double extendVolts) {
    io.setVoltageExtend(extendVolts);
  }

  private void stopIntake() {
    io.setVoltageIntake(0);
  }

  private void stopExtend() {
    io.setVoltageExtend(0);
  }

  private boolean isHopperCollapsed() {
    return io.isHopperCollapsed();
  }

  private boolean isStallingCollapse() {
    return Math.abs(inputs.extendVelocity) < 0.1 && inputs.extendVolts < -0.05;
  }

  private boolean isStallingExtend() {
    return Math.abs(inputs.extendVelocity) < 0.1 && inputs.extendVolts > 0.05;
  }

  public Command extend() {
    return Commands.run(
            () -> {
              setVoltageExtend(EXTEND_VOLTS);
            })
        .until(() -> inputs.getExtendPos >= EXTEND_POS)
        .finallyDo(interrupted -> stopExtend());
  }

  public Command collapse() {
    return Commands.run(
            () -> {
              setVoltageExtend(COLLAPSE_VOLTS);
            })
        .until(() -> isHopperCollapsed() || inputs.getExtendPos <= COLLAPSE_POS)
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

  public Command stopIntakeCommand() {
    return Commands.run(this::stopIntake);
  }

  public Command stopExtendingCommand() {
    return Commands.run(this::stopExtend);
  }

  public Command extendAndIntake() {
    return Commands.run(
            () -> {
              setVoltageIntake(INTAKE_VOLTS);
              setVoltageExtend(EXTEND_VOLTS);
            })
        .until(() -> inputs.getExtendPos >= EXTEND_POS)
        .finallyDo(interrupted -> stopExtend())
        .andThen(intake());
  }

  public Command collapseAndIntake() {
    return Commands.run(
            () -> {
              setVoltageIntake(INTAKE_VOLTS);
              setVoltageExtend(COLLAPSE_VOLTS);
            })
        .until(() -> isHopperCollapsed() || inputs.getExtendPos <= COLLAPSE_POS)
        .finallyDo(interrupted -> stopExtend())
        .andThen(intake());
  }

  public Command toPosExtend() {
    return Commands.run(() -> extendPos = EXTEND_POS)
        .until(() -> inputs.getExtendPos >= EXTEND_POS);
  }

  public Command toPosCollapse() {
    return Commands.run(() -> extendPos = COLLAPSE_POS)
        .until(() -> inputs.getExtendPos <= COLLAPSE_POS);
  }
}
