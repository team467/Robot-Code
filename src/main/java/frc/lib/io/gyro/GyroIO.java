package frc.lib.io.gyro;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** An abstract IO for reading a Gyro. */
public interface GyroIO {

  class GyroIOInputs implements LoggableInputs {

    /** Whether the gyro is connected */
    public boolean connected = false;
    /** The angle of the gyro in degrees */
    public double angle = 0.0;
    /** The rate of the gyro in degrees per second */
    public double rate = 0.0;

    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("Angle", angle);
      table.put("Rate", rate);
    }

    public void fromLog(LogTable table) {
      connected = table.getBoolean("Connected", connected);
      angle = table.getDouble("Angle", angle);
      rate = table.getDouble("Rate", rate);
    }
  }

  /**
   * Update the inputs of the gyro
   *
   * @param inputs The inputs to update
   */
  default void updateInputs(GyroIOInputs inputs) {
    inputs.connected = false;
    inputs.angle = 0.0;
    inputs.rate = 0.0;
  }
}
