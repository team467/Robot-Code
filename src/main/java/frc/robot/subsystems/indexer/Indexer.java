package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexConstants.FEEDUP_VOLT;
import static frc.robot.subsystems.indexer.IndexConstants.PRELOAD_VOLT;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    RobotState.getInstance().indexerHasFuel = inputs.ballAtLeftSwitch || inputs.ballAtRightSwitch;
    Logger.processInputs("Index", inputs);
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

  public boolean isLeftSwitchPressed() {
    return io.isLeftSwitchPressed();
  }

  public boolean isRightSwitchPressed() {
    return io.isRightSwitchPressed();
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

  public Command runPreloadSpeeds() {
    return Commands.run(
            () -> {
              setVoltage(PRELOAD_VOLT);
            },
            this)
        .finallyDo(this::stop)
        .withName("runPeriodic");
  }

  public Command reverse() {
    return Commands.run(
            () -> {
              setVoltage(-FEEDUP_VOLT);
            })
        .finallyDo(this::stop)
        .withName("reverse");
  }

  public Command stop() {
    return Commands.run(io::stop, this).withName("stop");
  }
}
