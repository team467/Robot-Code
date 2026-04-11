package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.extend.IntakeExtend;
import frc.robot.subsystems.intake.rollers.IntakeRollers;

public class Intake {
  private final IntakeRollers rollers;
  private final IntakeExtend extend;

  public Intake(IntakeRollers rollers, IntakeExtend extend) {
    this.rollers = rollers;
    this.extend = extend;
  }

  public Command extendToAngleAndIntake(double angle) {
    return Commands.parallel(extend.extendToAngle(angle), rollers.intake())
        .withName("extendToAngleAndIntake");
  }

  public Command extendToAngle(double angle) {
    return extend.extendToAngle(angle);
  }

  public Command holdAngleAndIntake(double angle) {
    return Commands.parallel(extend.holdAngle(angle), rollers.intake())
        .withName("holdAngleAndIntake");
  }

  // Jack's Chugga Chugga mode

  public Command shakeAndIntake() {
    return Commands.repeatingSequence(
            Commands.deadline(
                extend.extendToAngle(FUNNEL_POS + SHAKE_POS_OFFSET), rollers.intake()),
            Commands.deadline(
                extend.extendToAngle(FUNNEL_POS - SHAKE_POS_OFFSET), rollers.intake()))
        .withName("shakeAndIntake");
  }

  public Command shake() {
    return Commands.repeatingSequence(
            extend.extendToAngle(FUNNEL_POS + SHAKE_POS_OFFSET),
            extend.extendToAngle(FUNNEL_POS - SHAKE_POS_OFFSET))
        .withName("shake");
  }
}
