package first.robot.mechanisms.Intake;

import static first.robot.constants.IntakeConstants.*;

import first.robot.mechanisms.Intake.Extend.IntakeExtend;
import first.robot.mechanisms.Intake.Extend.IntakeExtendIO;
import first.robot.mechanisms.Intake.Rollers.IntakeRoller;
import first.robot.mechanisms.Intake.Rollers.IntakeRollerIO;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

public class Intake implements Mechanism {
  private final IntakeExtend extend;
  private final IntakeRoller roller;

  public Intake(IntakeExtendIO extendIO, IntakeRollerIO rollerIO) {
    extend = new IntakeExtend(extendIO);
    roller = new IntakeRoller(rollerIO);
  }

  public void applyOutputs() {
    extend.applyOutputs();
    roller.applyOutputs();
  }

  public Command intake() {
    return run(coroutine -> {
          coroutine.fork(extend.extend());
          coroutine.waitUntil(() -> extend.atPosition(EXTEND_POS));
          coroutine.fork(roller.intake());
          coroutine.park();
        })
        .named("Intake.Intake");
  }

  public Command outtake() {
    return run(coroutine -> {
          coroutine.fork(extend.extend());
          coroutine.waitUntil(() -> extend.atPosition(EXTEND_POS));
          coroutine.fork(roller.outtake());
          coroutine.park();
        })
        .named("Intake.Outtake");
  }

  public Command runRollers() {
    return run(coroutine -> {
          coroutine.fork(roller.intake());
          coroutine.park();
        })
        .named("Intake.RunRollers");
  }

  public Command stow() {
    return run(coroutine -> {
          coroutine.fork(roller.stop(), extend.collapse());
          coroutine.park();
        })
        .named("Intake.Stow");
  }

  public Command agitate() {
    return run(coroutine -> {
          coroutine.fork(roller.intake(), extend.shake(FUNNEL_POS, SHAKE_POS_OFFSET));
          coroutine.park();
        })
        .named("Intake.Agitate");
  }

  public boolean isDeployed() {
    return extend.isDeployed();
  }

  public boolean isIntaking() {
    return roller.isIntaking();
  }

  public Command extendToAngle(double position) {
    return run(coroutine -> {
          coroutine.fork(extend.toPosition(position));
          coroutine.park();
        })
        .named("Intake.ExtendToAngle");
  }

  public Command extendToAngleAndIntake(double position) {
    return run(coroutine -> {
          coroutine.fork(extend.toPosition(position), roller.intake());
          coroutine.park();
        })
        .named("Intake.ExtendToAngleAndIntake");
  }

  public Command runIntakeMotor() {
    return runRollers();
  }

  public Command outtakeRollers() {
    return run(coroutine -> {
          coroutine.fork(roller.outtake());
          coroutine.park();
        })
        .named("Intake.OuttakeRollers");
  }

  public Command slowlyBringInIntakeWithoutRollers() {
    return run(coroutine -> {
          coroutine.fork(roller.stop(), extend.slowCollapse());
          coroutine.park();
        })
        .named("Intake.SlowlyBringInWithoutRollers");
  }

  public Command slowlyBringInIntake() {
    return run(coroutine -> {
          coroutine.fork(roller.intake(), extend.slowCollapse());
          coroutine.park();
        })
        .named("Intake.SlowlyBringIn");
  }

  public Command slowlyBringInThenShake() {
    return run(coroutine -> {
          coroutine.fork(roller.intake(), extend.slowCollapseThenShake());
          coroutine.park();
        })
        .named("Intake.SlowlyBringInThenShake");
  }

  public Command stopIntakeCommand() {
    return run(coroutine -> {
          coroutine.fork(roller.stop());
          coroutine.park();
        })
        .named("Intake.StopRollers");
  }

  public Command runIntakeExtendVolts(double volts) {
    return run(coroutine -> {
          coroutine.fork(extend.runVolts(volts));
          coroutine.park();
        })
        .named("Intake.RunExtendVolts");
  }

  public Command stopExtendingCommand() {
    return run(coroutine -> {
          coroutine.fork(extend.stopCommand());
          coroutine.park();
        })
        .named("Intake.StopExtending");
  }

  public Command resetExtendPosition() {
    return run(coroutine -> coroutine.await(extend.resetPosition())).named("Intake.ResetExtend");
  }

  public Command home() {
    return run(coroutine -> {
          coroutine.fork(roller.stop());
          coroutine.await(extend.home());
        })
        .named("Intake.Home");
  }
}
