package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.extend.IntakeExtend;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.intake.rollers.IntakeRollers.State;

public class Intake {
  private final IntakeRollers rollers;
  private final IntakeExtend extend;

  /** Creates the intake facade from the roller and extension subsystems. */
  public Intake(IntakeRollers rollers, IntakeExtend extend) {
    this.rollers = rollers;
    this.extend = extend;
  }

  public IntakeRollers rollerInstance() {
    return this.rollers;
  }

  public IntakeExtend extendInstance() {
    return this.extend;
  }

  /** Extends to the requested angle while running the rollers inward. */
  public Command extendToAngleAndIntake(double angle) {
    return Commands.run(
            () -> {
              rollers.rollerState = State.RUNNING;
            })
        .withName("extendToAngleAndIntake");
  }

  /** Moves the intake extension to the requested angle. */
  public Command extendToAngle(double angle) {
    return extend.extendToAngle(angle);
  }

  /** Runs the intake rollers inward. */
  public Command runIntakeMotor() {
    return Commands.runEnd(
            () -> rollers.rollerState = State.RUNNING, () -> rollers.rollerState = State.IDLE)
        .withName("runIntakeMotor");
  }

  /** Collapses the intake slowly while keeping the rollers running. */
  public Command slowlyBringInIntake() {
    return Commands.parallel(
        Commands.run(
            () -> {
              rollers.rollerState = State.RUNNING;
            }),
        extend
            .runIntakeExtendVolts(SLOW_VOLTS)
            .until(extend::isHopperCollapsed)
            .andThen(extend.extendToAngle(COLLAPSE_POS).repeatedly()));
  }

  /** Collapses the intake slowly without moving the rollers. */
  public Command slowlyBringInIntakeWithoutRollers() {
    return extend
        .runIntakeExtendVolts(SLOW_VOLTS)
        .until(extend::isHopperCollapsed)
        .andThen(extend.extendToAngle(COLLAPSE_POS).repeatedly());
  }

  /** Alternates the extension around the funnel position while intaking game pieces. */
  public Command shakeAndIntake() {
    return Commands.repeatingSequence(
            Commands.deadline(
                extend.extendToAngle(FUNNEL_POS + SHAKE_POS_OFFSET), runIntakeMotor()),
            Commands.deadline(
                extend.extendToAngle(FUNNEL_POS - SHAKE_POS_OFFSET), runIntakeMotor()))
        .withName("shakeAndIntake");
  }

  /** Alternates the extension around the funnel position without running the rollers. */
  public Command shake() {
    return Commands.repeatingSequence(
            extend.extendToAngle(FUNNEL_POS + SHAKE_POS_OFFSET),
            extend.extendToAngle(FUNNEL_POS - SHAKE_POS_OFFSET))
        .withName("shake");
  }
}
