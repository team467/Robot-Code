package frc.robot.subsystems.indexer;

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

  private void setPercent(double indexPercent) {
    io.setPercent(indexPercent);
  }

  private void setVoltage(double indexVolts) {
    io.setVoltage(indexVolts);
  }

  private void stop() {
    io.setVoltage(0);
  }

  private boolean isSwitchPressed() {
    return io.isSwitchPressed();
  }

  public Command run() {
    return Commands.run(
        () -> {
          setVoltage(INDEX_VOLT);
        });
  }

  public Command stopCommand() {
    return Commands.run(
        () -> {
          stop();
        });
  }
}
