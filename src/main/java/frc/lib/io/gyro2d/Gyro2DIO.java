package frc.lib.io.gyro2d;

import org.littletonrobotics.junction.AutoLog;

/** An abstract IO for reading a Gyro. */
public interface Gyro2DIO {
  @AutoLog
  class Gyro2DIOInputs {

    /** Whether the gyro is connected */
    public boolean connected = false;
    /** The angle of the gyro in degrees */
    public double angle = 0.0;
    /** The rate of the gyro in degrees per second */
    public double rate = 0.0;
  }

  /**
   * Update the inputs of the gyro
   *
   * @param inputs The inputs to update
   */
  default void updateInputs(Gyro2DIOInputs inputs) {
    inputs.connected = false;
    inputs.angle = 0.0;
    inputs.rate = 0.0;
  }
}
