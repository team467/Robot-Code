package frc.lib.io.gyro3d;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class GyroPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Double> yaw;
  private final StatusSignal<Double> yawVelocity;
  private final StatusSignal<Double> pitchVelocity;
  private final StatusSignal<Double> rollVelocity;
  private final StatusSignal<Double> quatW;
  private final StatusSignal<Double> quatX;
  private final StatusSignal<Double> quatY;
  private final StatusSignal<Double> quatZ;

  public GyroPigeon2(int deviceID) {
    pigeon = new Pigeon2(deviceID);
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    pitchVelocity = pigeon.getAngularVelocityYWorld();
    rollVelocity = pigeon.getAngularVelocityXWorld();
    quatW = pigeon.getQuatW();
    quatX = pigeon.getQuatX();
    quatY = pigeon.getQuatY();
    quatZ = pigeon.getQuatZ();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                yaw, yawVelocity, pitchVelocity, rollVelocity, quatW, quatX, quatY, quatZ)
            .equals(StatusCode.OK);
    inputs.yaw = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.rollRate = Units.degreesToRadians(rollVelocity.getValueAsDouble());
    inputs.pitchRate = Units.degreesToRadians(pitchVelocity.getValueAsDouble());
    inputs.yawRate = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.rotation3d =
        new Rotation3d(
            new Quaternion(
                quatW.getValueAsDouble(),
                quatX.getValueAsDouble(),
                quatY.getValueAsDouble(),
                quatZ.getValueAsDouble()));
  }
}
