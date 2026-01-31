package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public static Timer stowTimer = new Timer();
  public static Timer extendTimer = new Timer();
  private boolean stalledCollapse = true;
  private boolean stalledExtend = false;
  private boolean isStowed = false;
  private boolean isExtended = false;
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

    if (!limitSwitchDisabled.getAsBoolean() && isHopperCollapsed()) {
      io.resetExtendEncoder();
    }

    if (limitSwitchDisabled.getAsBoolean()) {
      if (inputs.extendVolts > 0.1 && stalledExtend) {
        stalledExtend = false;
        isExtended = false;
      }
      if (inputs.extendVolts < -0.1 && stalledCollapse) {
        stalledCollapse = false;
        isStowed = false;
      }

      if (isStallingCollapse()) {
        stowTimer.start();
      } else {
        stowTimer.stop();
        stowTimer.reset();
      }

      if (stowTimer.get() > 0.05) {
        stalledCollapse = true;
        stowTimer.stop();
        stowTimer.reset();
      }

      if (isStallingExtend()) {
        extendTimer.start();
      } else {
        extendTimer.stop();
        extendTimer.reset();
      }

      if (extendTimer.get() > 0.05) {
        stalledExtend = true;
        extendTimer.stop();
        extendTimer.reset();
      }

      if (stalledCollapse && inputs.extendVolts == 0) {
        isStowed = true;
      }
      if (stalledExtend && inputs.extendVolts == 0) {
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

  private boolean isStallingCollapse() {
    return Math.abs(inputs.extendVelocity) < 0.1 && inputs.extendVolts < -0.1;
  }

  private boolean isStallingExtend() {
    return Math.abs(inputs.extendVelocity) < 0.1 && inputs.extendVolts > 0.1;
  }

  public Command voltageTest() {
    return Commands.run(
        () -> {
          setVoltageExtend(EXTEND_VOLTS);
          setVoltageIntake(INTAKE_VOLTS);
        });
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
                limitSwitchDisabled.getAsBoolean() ? isExtended : inputs.getExtendPos >= EXTEND_POS)
        .finallyDo(
            () -> {
              io.setPIDEnabled(false);
              stopExtend();
            });
  }

  public Command toPosCollapse() {
    return Commands.run(
            () -> {
              io.setPIDEnabled(true);
              io.goToPos(COLLAPSE_POS);
            })
        .until(
            () ->
                limitSwitchDisabled.getAsBoolean()
                    ? isStowed
                    : (isHopperCollapsed() || inputs.getExtendPos <= COLLAPSE_POS))
        .finallyDo(
            () -> {
              io.setPIDEnabled(false);
              stopExtend();
            });
  }
}
