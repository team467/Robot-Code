package frc.lib.io.gyro2d;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

/** A Gyro IO class that interacts with an ADIS16470. */
public class Gyro2DIOADIS16470 implements Gyro2DIO {

  private final ADIS16470_IMU gyro;

  public Gyro2DIOADIS16470() {
    this.gyro = new ADIS16470_IMU();
    gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kY);
    gyro.calibrate();
    gyro.reset();
  }

  @Override
  public void updateInputs(Gyro2DIOInputs inputs) {
    inputs.connected = true;
    inputs.angle = gyro.getAngle();
    inputs.rate = gyro.getRate();
  }
}
