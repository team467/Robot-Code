package first.robot.mechanisms.magicCarpet;

import first.robot.mechanisms.magicCarpet.MagicCarpetIO.MagicCarpetIOOutputMode;
import first.robot.mechanisms.magicCarpet.MagicCarpetIO.MagicCarpetIOOutputs;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

public class MagicCarpet implements Mechanism {
  private final MagicCarpetIO io;
  private final MagicCarpetIOInputsAutoLogged inputs = new MagicCarpetIOInputsAutoLogged();
  private final MagicCarpetIOOutputs outputs = new MagicCarpetIOOutputs();

  public MagicCarpet(MagicCarpetIO io) {
    this.io = io;
    Scheduler.getDefault().addPeriodic(this::updateInputs);
  }

  private void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("MagicCarpet", inputs);
  }

  public void applyOutputs() {
    io.applyOutputs(outputs);
  }

  private void runSpeed(double speed) {
    outputs.mode = MagicCarpetIOOutputMode.DUTY_CYCLE;
    outputs.targetSpeed = speed;
  }

  private void stopOutput() {
    outputs.mode = MagicCarpetIOOutputMode.STOP;
  }

  @Override
  public Command idle() {
    return run(coroutine -> {
          stopOutput();
          coroutine.park();
        })
        .withPriority(Command.LOWEST_PRIORITY)
        .named("MagicCarpet.Idle");
  }

  public Command runWhile(BooleanSupplier shouldRun) {
    return run(coroutine -> {
          while (true) {
            if (shouldRun.getAsBoolean()) {
              runSpeed(MagicCarpetConstants.BELT_SPEED);
            } else {
              stopOutput();
            }
            coroutine.yield();
          }
        })
        .withPriority(Command.LOWEST_PRIORITY)
        .named("MagicCarpet.RunWhile");
  }

  public Command run() {
    return run(coroutine -> {
          runSpeed(MagicCarpetConstants.BELT_SPEED);
          coroutine.park();
        })
        .named("start");
  }
}
