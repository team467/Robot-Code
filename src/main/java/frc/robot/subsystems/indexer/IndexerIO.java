
package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IndexerIO {
  @AutoLog
  class IndexerIOInputs {
    public double indexerVelocityRadPerSec;

    public double indexerVelocityMetersPerSec;

    public double indexerAppliedVolts;

    public double indexerCurrentAmps;

    public boolean indexerLimitSwitchPressed;
  }

  default void setIndexerVoltage(double volts) {}

  default void setIndexerPercentVelocity(double percent) {}

  default void updateInputs(IndexerIOInputs inputs) {}
}
