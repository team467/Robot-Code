package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final Timer stallExtendTimer = new Timer();
  private final Timer stallCollapseTimer = new Timer();

  private boolean stalledExtend = false;
  private boolean stalledCollapse = false;
  private boolean isStowed = false;
  private boolean isExtended = false;

  private double extendPos = 0;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final BooleanSupplier limitSwitchDisabled;

  public Intake(IntakeIO io, BooleanSupplier limitSwitchDisabled) {
    this.io = io;
    this.limitSwitchDisabled = limitSwitchDisabled;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (!limitSwitchDisabled.getAsBoolean() && isHopperCollapsed()) {
      io.resetExtendEncoder(0);
      isStowed = true;
      stalledCollapse = false;
    }

    io.extendToPosition(extendPos);

    if (limitSwitchDisabled.getAsBoolean()) {
      if (!stalledExtend && isStallingExtend()) {
        stallExtendTimer.start();
      } else {
        stallExtendTimer.stop();
        stallExtendTimer.reset();
      }

      if (!stalledExtend && stallExtendTimer.get() > STALL_TIME) {
        stalledExtend = true;
        stallExtendTimer.stop();
        stallExtendTimer.reset();
        io.resetExtendEncoder(EXTEND_POS);
        isExtended = true;
      }

      if (!stalledCollapse && isStallingCollapse()) {
        stallCollapseTimer.start();
      } else {
        stallCollapseTimer.stop();
        stallCollapseTimer.reset();
      }

      if (!stalledCollapse && stallCollapseTimer.get() > STALL_TIME) {
        stalledCollapse = true;
        stallCollapseTimer.stop();
        stallCollapseTimer.reset();
        io.resetExtendEncoder(0);
        isStowed = true;
      }

      if (isMoving()) {
        stalledExtend = false;
        stalledCollapse = false;
        isExtended = false;
        isStowed = false;
      }
    }

    Logger.recordOutput("Intake/StalledExtend", stalledExtend);
    Logger.recordOutput("Intake/StalledCollapse", stalledCollapse);
  }

  private void setPercentIntake(double intakePercent) {
    io.setPercentIntake(intakePercent);
  }

  private void setPercentExtend(double extendPercent) {
    io.setPercentExtend(extendPercent);
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

  private boolean isStallingExtend() {
    return Math.abs(inputs.extendVelocity) < STALL_VELOCITY && inputs.extendVolts > 0.5;
  }

  private boolean isStallingCollapse() {
    return Math.abs(inputs.extendVelocity) < STALL_VELOCITY && inputs.extendVolts < -0.5;
  }

  private boolean isMoving() {
    return Math.abs(inputs.extendVelocity) > STALL_VELOCITY;
  }

  public Command extend() {
    return Commands.run(() -> setVoltageExtend(EXTEND_VOLTS))
        .until(() -> inputs.getExtendPos >= EXTEND_POS)
        .finallyDo(interrupted -> stopExtend());
  }

  public Command collapse() {
    return Commands.run(() -> setVoltageExtend(COLLAPSE_VOLTS))
        .until(() -> isHopperCollapsed() || inputs.getExtendPos <= COLLAPSE_POS)
        .finallyDo(interrupted -> stopExtend());
  }

  public Command intake() {
    return Commands.run(() -> setVoltageIntake(INTAKE_VOLTS))
        .finallyDo(interrupted -> stopIntake());
  }

  public Command outtake() {
    return Commands.run(() -> setVoltageIntake(OUTTAKE_VOLTS))
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
