package frc.robot.subsystems.indexer;

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

  public void setPercent(double percent) {
    io.setPercent(percent);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void stop() {
    io.stop();
  }

  public boolean isSwitchPressed() {
    return io.isSwitchPressed();
  }
}
