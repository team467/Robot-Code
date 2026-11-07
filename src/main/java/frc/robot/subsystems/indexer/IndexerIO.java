package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  class IndexerIOInputs {

    public double feedUpPercentOutput = 0.0;
    public double feedUpVolts = 0.0;
    public double feedUpAmps = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void setPercent(double indexPercent, double feedUpPercent) {}

  default void setVoltage(double feedUpVolts) {}

  default void stop() {}
}
