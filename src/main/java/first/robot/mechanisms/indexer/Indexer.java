package first.robot.mechanisms.indexer;

import static first.robot.mechanisms.indexer.IndexerConstants.FEEDUP_VOLT;

import first.robot.mechanisms.indexer.IndexerIO.IndexerIOOutputMode;
import first.robot.mechanisms.indexer.IndexerIO.IndexerIOOutputs;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

public class Indexer implements Mechanism {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final IndexerIOOutputs outputs = new IndexerIOOutputs();

  public Indexer(IndexerIO io) {
    this.io = io;
    Scheduler.getDefault().addPeriodic(this::updateInputs);
  }

  private void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Index", inputs);
  }

  public void applyOutputs() {
    io.applyOutputs(outputs);
  }

  private void setVoltage(double feedUpVolts) {
    outputs.mode = IndexerIOOutputMode.VOLTAGE;
    outputs.targetVolts = feedUpVolts;
  }

  private void stopOutput() {
    outputs.mode = IndexerIOOutputMode.STOP;
  }

  public double getVoltage() {
    return inputs.feedUpVolts;
  }

  @AutoLogOutput(key = "Indexer/Running")
  public boolean isRunning() {
    return inputs.feedUpVolts > 0;
  }

  @Override
  public Command idle() {
    return run(coroutine -> {
          stopOutput();
          coroutine.park();
        })
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Indexer.Idle");
  }

  public Command run() {
    return run(coroutine -> {
          setVoltage(FEEDUP_VOLT);
          coroutine.park();
        })
        .named("run");
  }

  public Command reverse() {
    return run(coroutine -> {
          setVoltage(-FEEDUP_VOLT);
          coroutine.park();
        })
        .named("reverse");
  }

  public Command stop() {
    return run(coroutine -> {
          stopOutput();
          coroutine.park();
        })
        .named("stop");
  }
}
