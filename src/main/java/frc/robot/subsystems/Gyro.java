package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
    ADIS16470_IMU gyro = new ADIS16470_IMU();

    public Gyro() {
        super();
    
        gyro.calibrate();
        gyro.setYawAxis(IMUAxis.kY);
        Shuffleboard.getTab("Main").add("asd", gyro);
    }

    public void calibrate() {
        gyro.calibrate();
    }

    public void reset() {
        gyro.reset();
    }

    public double getAngle() {
        return gyro.getAngle();
    }

    public double getRate() {
        return gyro.getRate();
    }

    public double getNextAngle() {
        return getAngle() + (getRate() * 0.02);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Angle", () -> gyro.getAngle(), null);
        builder.addDoubleProperty("Rate", () -> gyro.getRate(), null);
    }
}
