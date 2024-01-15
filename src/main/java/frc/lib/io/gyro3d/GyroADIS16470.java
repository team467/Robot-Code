package frc.lib.io.gyro3d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class GyroADIS16470 implements GyroIO {
    private final ADIS16470_IMU gyro;

    public GyroADIS16470() {
        this.gyro = new ADIS16470_IMU();
        gyro.calibrate();
        gyro.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yaw = Rotation2d.fromDegrees(gyro.getAngle(ADIS16470_IMU.IMUAxis.kYaw));
        inputs.rollRate = gyro.getRate(ADIS16470_IMU.IMUAxis.kRoll);
        inputs.pitchRate = gyro.getRate(ADIS16470_IMU.IMUAxis.kPitch);
        inputs.yawRate = gyro.getRate(ADIS16470_IMU.IMUAxis.kYaw);
        inputs.rotation3d = new Rotation3d(
                Units.degreesToRadians(gyro.getAngle(ADIS16470_IMU.IMUAxis.kRoll)),
                Units.degreesToRadians(gyro.getAngle(ADIS16470_IMU.IMUAxis.kPitch)),
                Units.degreesToRadians(gyro.getAngle(ADIS16470_IMU.IMUAxis.kYaw))
        );
    }
}
