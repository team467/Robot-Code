package frc.lib.io.gyro2d;

import edu.wpi.first.wpilibj.ADIS16448_IMU;

/** A Gyro IO class that interacts with an ADIS16448. */
public class Gyro2DIOADIS16448 implements Gyro2DIO {

  private final ADIS16448_IMU gyro;

  public Gyro2DIOADIS16448() {
    this.gyro = new ADIS16448_IMU();
    gyro.setYawAxis(ADIS16448_IMU.IMUAxis.kY);
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
