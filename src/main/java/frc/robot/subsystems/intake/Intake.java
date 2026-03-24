package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.extend.IntakeExtend;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import org.littletonrobotics.junction.Logger;

// TODO:rewrite this for v1.5
public class Intake {
  private final IntakeRollers rollers;
  private final IntakeExtend extend;

  public Intake(IntakeRollers rollers, IntakeExtend extend) {
    this.rollers = rollers;
    this.extend = extend;
  }

  public void periodic() {
    Logger.processInputs("Intake", inputs);
  }

  public Command extendToAngleAndIntake(double angle) {
    return Commands.run(
            () -> {
              rollers.setVoltageIntake(INTAKE_VOLTS);
              extend.setVoltageExtend(EXTEND_VOLTS);
            })
        .until(() -> inputs.getExtendPos >= EXTEND_POS)
        .finallyDo(interrupted -> extend.stopExtend())
        .andThen(rollers.intake())
        .withName("extendAndIntake");
  }

  public Command extendToAngle(double angle) {
    return Commands.run(
            () -> {
              io.setVoltageIntake(INTAKE_VOLTS);
              io.setVoltageExtend(COLLAPSE_VOLTS);
            })
        .until(() -> extend.isHopperCollapsed() || inputs.getExtendPos <= COLLAPSE_POS)
        .finallyDo(interrupted -> extend.stopExtend())
        .andThen(rollers.intake())
        .withName("collapseAndIntake");
  }
}
