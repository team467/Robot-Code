package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeRollers rollers;
  private final IntakeExtend extend;

  /**
   * Constructor for the Intake subsystem
   *
   * @param io The IO implementation to use
   */
  public Intake(IntakeIO io, IntakeRollers rollers, IntakeExtend extend) {
    this.io = io;
    this.rollers = rollers;
    this.extend = extend;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command extendAndIntake() {
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

  public Command collapseAndIntake() {
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
