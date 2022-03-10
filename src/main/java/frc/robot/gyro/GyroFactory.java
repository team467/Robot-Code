package frc.robot.gyro;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.utilities.IMUAxis;
import frc.robot.utilities.IMUType;

public class GyroFactory {

  private GyroFactory() {
    throw new IllegalStateException("Utility class");
  }

  /**
   * Creates a new Gyro
   *
   * @param type The type of Gyro being used
   * @param yawAxis The axis to check on the Gyro
   * @return A gyro
   * @see Gyro
   */
  public static Gyro create(IMUType type, IMUAxis yawAxis) {
    switch (type) {
      case ADIS16448:
        ADIS16448 adis16448 = new ADIS16448();
        switch (yawAxis) {
          case kX:
            adis16448.setYawAxis(ADIS16448_IMU.IMUAxis.kX);
            break;
          case kY:
            adis16448.setYawAxis(ADIS16448_IMU.IMUAxis.kY);
            break;
          case kZ:
            adis16448.setYawAxis(ADIS16448_IMU.IMUAxis.kZ);
            break;
        }
        adis16448.calibrate();
        adis16448.reset();
        return adis16448;
      case ADIS16470:
        ADIS16470 adis16470 = new ADIS16470();
        switch (yawAxis) {
          case kX:
            adis16470.setYawAxis(ADIS16470_IMU.IMUAxis.kX);
            break;
          case kY:
            adis16470.setYawAxis(ADIS16470_IMU.IMUAxis.kY);
            break;
          case kZ:
            adis16470.setYawAxis(ADIS16470_IMU.IMUAxis.kZ);
            break;
        }
        adis16470.calibrate();
        adis16470.reset();
        return adis16470;
    }
    return null;
  }

  /**
   * Creates a new Gyro
   *
   * @param type The type of Gyro being used
   * @return A gyro
   */
  public static Gyro create(IMUType type) {
    return create(type, IMUAxis.kX);
  }
}
