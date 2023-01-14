package frc.lib.io.gyro;

import edu.wpi.first.wpilibj.ADIS16448_IMU;

/** A Gyro IO class that interacts with an ADIS16448. */
public class GyroIOADIS16448 implements GyroIO {

  private final ADIS16448_IMU gyro;

  public GyroIOADIS16448() {
    this.gyro = new ADIS16448_IMU();
    gyro.setYawAxis(ADIS16448_IMU.IMUAxis.kY);
    gyro.calibrate();
    gyro.reset();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.angle = gyro.getAngle();
    inputs.rate = gyro.getRate();
  }
}
