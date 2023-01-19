package frc.lib.io.gyro3d;

import org.littletonrobotics.junction.AutoLog;

public interface IMUIO {
  @AutoLog
  class IMUIOInputs {
    public boolean connected = false;
    /** CCW positive, degrees */
    public double roll = 0.0;
    /** CCW positive, degrees */
    public double yaw = 0.0;
    /** CCW positive, degrees */
    public double pitch = 0.0;
    /** CCW positive, degrees */
    public double rollRate = 0.0;
    /** CCW positive, degrees */
    public double yawRate = 0.0;
    /** CCW positive, degrees */
    public double pitchRate = 0.0;
    /** [x,y,z] */
    public double[] gravVector = new double[] {0, 0, 1};
  }

  /**
   * Update the inputs of the IMU
   *
   * @param inputs The inputs to update
   */
  default void updateInputs(IMUIOInputs inputs) {}
}
