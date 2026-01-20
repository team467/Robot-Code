package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexConstants.FEEDUP_VOLT;
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

  private void setVoltage(double indexVolt, double feedUpVolt) {
    io.setVoltage(indexVolt, feedUpVolt);
  }

  private void stop() {
    io.setVoltage(0, 0);
  }

  private void isSwitchPressed() {
    io.isSwitchPressed();
  }

  public Command start() {
    return Commands.run(
        () -> {
          setVoltage(INDEX_VOLT, FEEDUP_VOLT);
        });
  }

  public Command stopCommand() {
    return Commands.run(
        () -> {
          stop();
        });
  }
}
