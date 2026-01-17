package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  class IndexerIOInputs {

    public double percentOutput = 0.0;
    public double volts = 0.0;
    public double amps = 0.0;
    public boolean ballAtSwitch = false;
    // public boolean ballAtSwitch2 = false;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void setPercent(double indexPercent, double feedUpPercent) {}

  default void setVoltage(double volts, double volts2) {}

  default void stop() {}

  default boolean isSwitchPressed() {
    return false;
  }
}
