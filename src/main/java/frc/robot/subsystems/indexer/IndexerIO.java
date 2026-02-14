package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  class IndexerIOInputs {

    public double indexPercentOutput = 0.0;
    public double feedUpPercentOutput = 0.0;
    public double indexVolts = 0.0;
    public double feedUpVolts = 0.0;
    public double indexAmps = 0.0;
    public double feedUpAmps = 0.0;
    public boolean ballAtLeftSwitch = false;
    public boolean ballAtRightSwitch = false;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void setPercent(double indexPercent, double feedUpPercent) {}

  default void setVoltage(double indexVolts, double feedUpVolts) {}

  default void stop() {}

  default boolean isLeftSwitchPressed() {
    return false;
  }

  default boolean isRightSwitchPressed() {
    return false;
  }
}
