package frc.lib.io.lidar;

import org.littletonrobotics.junction.AutoLog;

public interface LidarIO {
  @AutoLog
  class LidarIOInputs {
    public double distance = 0.0;
  }
  default void updateInputs(LidarIOInputs inputs) {}
}
