package frc.lib.io.gyro3d;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

public class IMUPigeon2 implements IMUIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Double> yaw;
  private final StatusSignal<Double> pitch;
  private final StatusSignal<Double> roll;
  private final StatusSignal<Double> yawVelocity;
  private final StatusSignal<Double> pitchVelocity;
  private final StatusSignal<Double> rollVelocity;
  private final StatusSignal<Double> gravVectorX;
  private final StatusSignal<Double> gravVectorY;
  private final StatusSignal<Double> gravVectorZ;

  public IMUPigeon2(int deviceID) {
    pigeon = new Pigeon2(deviceID);
    yaw = pigeon.getYaw();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    yawVelocity = pigeon.getAngularVelocityZ();
    pitchVelocity = pigeon.getAngularVelocityY();
    rollVelocity = pigeon.getAngularVelocityX();
    gravVectorX = pigeon.getGravityVectorX();
    gravVectorY = pigeon.getGravityVectorY();
    gravVectorZ = pigeon.getGravityVectorZ();
  }

  @Override
  public void updateInputs(IMUIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                yaw,
                pitch,
                roll,
                yawVelocity,
                pitchVelocity,
                rollVelocity,
                gravVectorX,
                gravVectorY,
                gravVectorZ)
            .equals(StatusCode.OK);
    inputs.roll = roll.getValueAsDouble();
    inputs.pitch = pitch.getValueAsDouble();
    inputs.yaw = yaw.getValueAsDouble();
    inputs.rollRate = rollVelocity.getValueAsDouble();
    inputs.pitchRate = pitchVelocity.getValueAsDouble();
    inputs.yawRate = yawVelocity.getValueAsDouble();
    inputs.gravVector =
        new double[] {
          gravVectorX.getValueAsDouble(),
          gravVectorY.getValueAsDouble(),
          gravVectorZ.getValueAsDouble()
        };
  }
}
