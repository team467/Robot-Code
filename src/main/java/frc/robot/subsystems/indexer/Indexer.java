package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexConstants.FEEDUP_VOLT;
// import static frc.robot.subsystems.indexer.IndexConstants.FEEDUP_VOLT;
import static frc.robot.subsystems.indexer.IndexConstants.INDEX_VOLT;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    Logger.processInputs("Index", inputs);
  }

  private void setPercent(double indexPercent, double feedUpPercent) {
    io.setPercent(indexPercent, feedUpPercent);
  }

  private void setVoltage(double indexVolts, double feedUpVolts) {
    io.setVoltage(indexVolts, feedUpVolts);
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
              setVoltage(INDEX_VOLT, FEEDUP_VOLT);
            })
        .withName("IndexerRun");
  }

  public Command reverse() {
    return Commands.run(
            () -> {
              setVoltage(-INDEX_VOLT, -FEEDUP_VOLT);
            })
        .withName("IndexerReverse");
  }

  public Command stop() {
    return Commands.run(io::stop).withName("IndexerStop");
  }
}
