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
    return Commands.parallel(extend.extendToAngle(angle), runIntakeMotor())
        .withName("extendToAngleAndIntake");
  }

  public Command extendToAngle(double angle) {
    return extend.extendToAngle(angle);
  }

  public Command runIntakeMotor() {
    return rollers.intake();
  }

  public Command slowlyBringInIntake() {
    return Commands.parallel(rollers.intake(), extend.runIntakeExtendVolts(SLOW_VOLTS))
        .andThen(extend.extendToAngle(COLLAPSE_POS).repeatedly());
  }

  // Jack's Chugga Chugga mode
  public Command shakeAndIntake() {
    return Commands.repeatingSequence(
            Commands.deadline(
                extend.extendToAngle(FUNNEL_POS + SHAKE_POS_OFFSET), runIntakeMotor()),
            Commands.deadline(
                extend.extendToAngle(FUNNEL_POS - SHAKE_POS_OFFSET), runIntakeMotor()))
        .withName("shakeAndIntake");
  }

  public Command shake() {
    return Commands.repeatingSequence(
            extend.extendToAngle(FUNNEL_POS + SHAKE_POS_OFFSET),
            extend.extendToAngle(FUNNEL_POS - SHAKE_POS_OFFSET))
        .withName("shake");
  }
}
