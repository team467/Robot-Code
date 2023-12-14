package frc.lib.io.gyro3d;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class GyroPigeon2 implements GyroIO {
  private final WPI_Pigeon2 pigeon;
  double[] rate = new double[3];
  double[] quaternion = new double[3];

  public GyroPigeon2(int deviceID) {
    pigeon = new WPI_Pigeon2(deviceID);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = pigeon.getLastError() == ErrorCode.OK;
    inputs.yaw = Rotation2d.fromDegrees(pigeon.getYaw());
    pigeon.get6dQuaternion(quaternion);
    Quaternion quaternionC =
        new Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    inputs.rotation3d = new Rotation3d(quaternionC);
    pigeon.getRawGyro(rate);
    inputs.rollRatePerSec = Units.degreesToRadians(rate[0]);
    inputs.pitchRatePerSec = Units.degreesToRadians(rate[1]);
    inputs.yawRatePerSec = Units.degreesToRadians(rate[2]);
  }
}
