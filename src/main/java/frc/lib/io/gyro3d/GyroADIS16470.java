package frc.lib.io.gyro3d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import frc.robot.subsystems.drive.OdometryThread;
import java.util.OptionalDouble;
import java.util.Queue;

public class GyroADIS16470 implements GyroIO {
  private final ADIS16470_IMU gyro;

  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroADIS16470() {
    this.gyro = new ADIS16470_IMU();
    gyro.calibrate();
    gyro.reset();

    // TODO: how do i make yaw go the odometry speed??? hopefully we never use this IMU ever again
    yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        OdometryThread.getInstance()
            .registerSignal(
                () -> {
                  boolean valid = gyro.isConnected();
                  if (valid) {
                    return OptionalDouble.of(gyro.getAngle(ADIS16470_IMU.IMUAxis.kYaw));
                  } else {
                    return OptionalDouble.empty();
                  }
                });
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.yaw = Rotation2d.fromDegrees(gyro.getAngle(ADIS16470_IMU.IMUAxis.kYaw));
    inputs.rollRate = gyro.getRate(ADIS16470_IMU.IMUAxis.kRoll);
    inputs.pitchRate = gyro.getRate(ADIS16470_IMU.IMUAxis.kPitch);
    inputs.yawRate = gyro.getRate(ADIS16470_IMU.IMUAxis.kYaw);
    inputs.rotation3d =
        new Rotation3d(
            Units.degreesToRadians(gyro.getAngle(ADIS16470_IMU.IMUAxis.kRoll)),
            Units.degreesToRadians(gyro.getAngle(ADIS16470_IMU.IMUAxis.kPitch)),
            Units.degreesToRadians(gyro.getAngle(ADIS16470_IMU.IMUAxis.kYaw)));

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
