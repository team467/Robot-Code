package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.COLLAPSE_POS;
import static frc.robot.subsystems.intake.IntakeConstants.COLLAPSE_VOLTS;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_POS;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_VOLTS;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_VOLTS;
import static frc.robot.subsystems.intake.IntakeConstants.OUTTAKE_VOLTS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public static Timer stowTimer = new Timer();
  private boolean stalled = true;
  private boolean isStowed = false;
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
    if (limitSwitchDisabled.getAsBoolean()) {
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

      if (stalled && inputs.extendVolts == 0) {
        isStowed = true;
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
    io.setVoltageIntake(extendVolts);
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

  private boolean isSlipping() {
    return Math.abs(inputs.extendVelocity) < 0.1 && inputs.extendVolts < -0.1;
  }

  public Command extend() {
    return Commands.run(
            () -> {
              setVoltageExtend(EXTEND_VOLTS);
              toPosExtend();
            })
        // .until(() -> !isHopperCollapsed())
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
    return Commands.run(
            () -> {
              setVoltageIntake(EXTEND_VOLTS);
              setVoltageExtend(COLLAPSE_VOLTS);
              toPosExtend();
            })
        // .until(inputs.getExtendPos == EXTEND_POS)
        .finallyDo(interrupted -> stopExtend())
        .andThen(intake());
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
              toPosCollapse();
            })
        // .until(this::isHopperCollapsed)
        .finallyDo(interrupted -> stopExtend());
  }

  public Command collapseAndIntake() {
    return Commands.run(
            () -> {
              setVoltageIntake(INTAKE_VOLTS);
              setVoltageExtend(COLLAPSE_VOLTS);
              toPosCollapse();
            })
        // .until(this::isHopperCollapsed)
        .finallyDo(interrupted -> stopExtend())
        .andThen(intake());
  }

  public Command toPosExtend() {
    return Commands.run(
            () -> {
              io.setPIDEnabled(true);
              io.goToPos(EXTEND_POS);
            })
        .until(
            () ->
                (isSlipping() && !limitSwitchDisabled.getAsBoolean())
                    || inputs.getExtendPos == EXTEND_POS)
        .finallyDo(() -> io.setPIDEnabled(false));
  }

  public Command toPosCollapse() {
    return Commands.run(
            () -> {
              io.setPIDEnabled(true);
              io.goToPos(COLLAPSE_POS);
            })
        .until(
            () ->
                isHopperCollapsed()
                    || inputs.getExtendPos == COLLAPSE_POS
                    || (!limitSwitchDisabled.getAsBoolean() && isStowed))
        .finallyDo(() -> io.setPIDEnabled(false));
  }
}
