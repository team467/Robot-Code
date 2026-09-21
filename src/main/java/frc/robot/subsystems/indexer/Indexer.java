package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.FEEDUP_VOLT;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Indexer implements Mechanism {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Index", inputs);
    RobotState.getInstance().indexerRunning = inputs.feedUpVolts > 0;
  }

  private void setPercent(double indexPercent, double feedUpPercent) {
    io.setPercent(indexPercent, feedUpPercent);
  }

  private void setVoltage(double feedUpVolts) {
    io.setVoltage(feedUpVolts);
  }

  public double getVoltage() {
    return inputs.feedUpVolts;
  }

  public Command run() {
    return Commands.run(
            () -> {
              setVoltage(FEEDUP_VOLT);
            },
            this)
        .finallyDo(this::stop)
        .withName("run");
  }

  public Command reverse() {
    return Commands.run(
            () -> {
              setVoltage(-FEEDUP_VOLT);
            },
            this)
        .finallyDo(this::stop)
        .withName("reverse");
  }

  public Command stop() {
    return Commands.run(io::stop, this).withName("stop");
  }
}
