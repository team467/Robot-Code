package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase implements edu.wpi.first.wpilibj.interfaces.Gyro{
    ADIS16470_IMU gyro = new ADIS16470_IMU();

    public Gyro() {
        super();
    
        gyro.setYawAxis(IMUAxis.kY);
        gyro.calibrate();
        
        Shuffleboard.getTab("Main").add("asd", gyro);
    }

    @Override
    public void calibrate() {
        gyro.calibrate();
    }

    @Override
    public void reset() {
        gyro.reset();
    }

    @Override
    public double getAngle() {
        return gyro.getAngle();
    }

    @Override
    public double getRate() {
        return gyro.getRate();
    }

    @Override
    public void close() throws Exception {
        gyro.close();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Angle", () -> gyro.getAngle(), null);
        builder.addDoubleProperty("Rate", () -> gyro.getRate(), null);
    }
}
