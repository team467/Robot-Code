package frc.robot.gyro;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class ADIS16448 extends ADIS16448_IMU implements Gyro {
    @Override
    public void calibrate() {
        super.calibrate();
    }

    @Override
    public void reset() {
        super.reset();
    }

    @Override
    public double getAngle() {
        return super.getAngle();
    }

    @Override
    public double getRate() {
        return super.getRate();
    }

    @Override
    public void close() {
        super.close();
    }
}