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

  public Command extendToAngleAndIntake(double angle) {
    return extend.extendToAngle(angle)
        .andThen(
            Commands.parallel(
                extend.holdAngle(angle),
                rollers.intake()
            )
        )
        .withName("extendToAngleAndIntake");
  }
}



// Jack's Chugga Chugga mode

//  public Command shakeAndIntake() {
//    return Commands.repeatingSequence(
//            Commands.runOnce(
//                () ->
//                    Commands.deadline(
//                        moveToAnglePrivate(FUNNEL_POS + SHAKE_POS_OFFSET), intakePrivate()),
//                this),
//            Commands.runOnce(
//                () ->
//                    Commands.deadline(
//                        moveToAnglePrivate(FUNNEL_POS + SHAKE_POS_OFFSET), intakePrivate()),
//                this))
//        .withName("shakeAndIntake");
//  }

// private because it doesn't have requirements and therefore it shouldn't be called beyond the
// subsystem
// itself

