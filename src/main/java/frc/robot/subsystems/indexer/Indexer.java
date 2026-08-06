package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.FEEDUP_VOLT;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public enum State {
    IDLE,
    RUNNING,
    REVERSE
  }

  public State indexerState = State.IDLE;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Index", inputs);
    switch (indexerState) {
      case IDLE -> setVoltage(0.0);
      case RUNNING -> setVoltage(FEEDUP_VOLT);
      case REVERSE -> setVoltage(-FEEDUP_VOLT);
    }
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
    return runEnd(() -> indexerState = State.RUNNING, () -> indexerState = State.IDLE);
  }

  public Command stop() {
    return runOnce(() -> indexerState = State.IDLE);
  }

  public Command reverse() {
    return runEnd(() -> indexerState = State.REVERSE, () -> indexerState = State.IDLE);
  }

  public boolean isRunning() {
    return indexerState == State.RUNNING;
  }

  // CAUTION: Do not use unless absolutely necessary, may cause unexpected behavior
  public void overrideState(State newState) {
    indexerState = newState;
  }
}
