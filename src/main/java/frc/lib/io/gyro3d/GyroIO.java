package frc.lib.io.gyro3d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yaw = new Rotation2d();
    public Rotation3d rotation3d = new Rotation3d();
    public double rollRate = 0.0;
    public double yawRate = 0.0;
    public double pitchRate = 0.0;
  }

  /**
   * Update the inputs of the IMU
   *
   * @param inputs The inputs to update
   */
  default void updateInputs(GyroIOInputs inputs) {}
}
