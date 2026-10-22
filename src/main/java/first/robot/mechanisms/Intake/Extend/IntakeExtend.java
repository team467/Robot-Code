package first.robot.mechanisms.Intake.Extend;

import static first.robot.constants.IntakeConstants.*;

import first.robot.mechanisms.Intake.Extend.IntakeExtendIO.ExtendIOOutputMode;
import first.robot.mechanisms.Intake.Extend.IntakeExtendIO.IntakeExtendIOOutputs;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

public class IntakeExtend implements Mechanism {
  private final IntakeExtendIO io;
  private final IntakeExtendIOInputsAutoLogged inputs = new IntakeExtendIOInputsAutoLogged();
  private final IntakeExtendIOOutputs outputs = new IntakeExtendIOOutputs();

  public IntakeExtend(IntakeExtendIO io) {
    this.io = io;
    Scheduler.getDefault().addPeriodic(this::updateInputs);
  }

  private void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeExtend", inputs);
  }

  public void applyOutputs() {
    io.applyOutputs(outputs);
  }

  private void runPosition(double position) {
    outputs.mode = ExtendIOOutputMode.POSITION;
    outputs.targetPosition = position;
  }

  private void runVoltage(double volts) {
    outputs.mode = ExtendIOOutputMode.VOLTAGE;
    outputs.targetVolts = volts;
  }

  private void stop() {
    outputs.mode = ExtendIOOutputMode.STOP;
  }

  public Command extend() {
    return run(coroutine -> {
          runPosition(EXTEND_POS);
          coroutine.park();
        })
        .named("extend intake");
  }

  public Command collapse() {
    return run(coroutine -> {
          runPosition(COLLAPSE_POS);
          coroutine.park();
        })
        .named("collapse intake");
  }

  public Command home() {
    return run(coroutine -> {
          try {
            runVoltage(HOME_VOLTAGE);
            coroutine.waitUntil(this::getLimitSwitchState);
          } finally {
            stop();
          }
        })
        .named("home intake");
  }

  public Command resetPosition() {
    return run(coroutine -> io.resetPosition()).named("reset intake extend position");
  }

  public Command toPosition(double position) {
    return run(coroutine -> {
          runPosition(position);
          coroutine.park();
        })
        .named("intake to position");
  }

  public Command runVolts(double volts) {
    return run(coroutine -> {
          runVoltage(volts);
          coroutine.park();
        })
        .whenCanceled(this::stop)
        .named("run intake extend volts");
  }

  public Command stopCommand() {
    return run(coroutine -> {
          stop();
          coroutine.park();
        })
        .named("stop extending");
  }

  public Command slowCollapse() {
    return run(coroutine -> {
          runVoltage(SLOW_VOLTS);
          coroutine.waitUntil(() -> atPosition(COLLAPSE_POS, SAFETY_TOLERANCE));
          runPosition(COLLAPSE_POS);
          coroutine.park();
        })
        .whenCanceled(this::stop)
        .named("slow collapse intake");
  }

  public boolean isDeployed() {
    return Math.abs(inputs.position - EXTEND_POS) < Math.abs(inputs.position - COLLAPSE_POS);
  }

  public Command shake(double center, double offset) {
    return run(coroutine -> shakeForever(coroutine, center, offset)).named("shake intake");
  }

  public Command slowCollapseThenShake() {
    return run(coroutine -> {
          runVoltage(SLOW_VOLTS);
          coroutine.waitUntil(() -> atPosition(COLLAPSE_POS, SAFETY_TOLERANCE));
          shakeForever(coroutine, COLLAPSE_POS - SHAKE_POS_OFFSET, SHAKE_POS_OFFSET);
        })
        .whenCanceled(this::stop)
        .named("slow collapse then shake intake");
  }

  private void shakeForever(Coroutine coroutine, double center, double offset) {
    double target = center + offset;
    while (true) {
      runPosition(target);
      final double currentTarget = target;
      coroutine.waitUntil(() -> atPosition(currentTarget, offset / 2.0));
      target = target > center ? center - offset : center + offset;
    }
  }

  public boolean atPosition(double position) {
    return atPosition(position, POSITION_TOLERANCE);
  }

  public boolean atPosition(double position, double tolerance) {
    return Math.abs(inputs.position - position) <= tolerance;
  }

  public boolean getLimitSwitchState() {
    return inputs.limitSwitchPressed;
  }
}
