package frc.lib.io.lidar;

import org.littletonrobotics.junction.AutoLog;

public interface LidarIO {
  @AutoLog
  class LidarIOInputs {
    public int distanceMeters = 0;
    public int strength = 0;
    public double temperatureCelcius = 0.0;
  }

  default void updateInputs(LidarIOInputs inputs) {}
}
