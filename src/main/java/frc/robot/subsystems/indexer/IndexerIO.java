package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  class IndexerIOInputs {
    public double indexerVelocityRadPerSec;

    public double indexerAppliedVolts;

    public double indexerCurrentAmps;

    public boolean indexerLimitSwitchLeftPressed;
    public boolean indexerLimitSwitchRightPressed;

  }

  default void setIndexerVoltage(double volts) {}

  default void setIndexerPercentVelocity(double percent) {}

  default void updateInputs(IndexerIOInputs inputs) {}
}
