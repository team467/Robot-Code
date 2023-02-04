package frc.lib.io.gyro3d;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class IMUPigeon2 implements IMUIO {
  private final WPI_Pigeon2 pigeon;
  double[] rate = new double[3];
  double[] gravVector = new double[3];

  public IMUPigeon2(int deviceID) {
    pigeon = new WPI_Pigeon2(deviceID);
  }

  @Override
  public void updateInputs(IMUIOInputs inputs) {
    inputs.connected = pigeon.getLastError() == ErrorCode.OK;
    inputs.roll = pigeon.getRoll();
    inputs.pitch = pigeon.getPitch();
    inputs.yaw = pigeon.getYaw();
    pigeon.getRawGyro(rate);
    inputs.rollRate = rate[0];
    inputs.pitchRate = rate[1];
    inputs.yawRate = rate[2];
    pigeon.getGravityVector(gravVector);
    inputs.gravVector = gravVector;
  }
}
