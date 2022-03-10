package frc.robot.motors;

import com.revrobotics.CANSparkMaxLowLevel;

public class MotorControllerFactory {

  private MotorControllerFactory() {
    throw new IllegalStateException("Utility class");
  }

  /**
   * Creates a new Motor Controller
   *
   * @param motorID the motor ID
   * @param type the type of motor, such as TALON_SRX, or SPARK_MAX_BRUSHLESS
   * @return A controller based on the motor type
   */
  public static MotorControllerEncoder create(int motorID, MotorType type) {
    switch (type) {
      case TALON_SRX:
        return new TalonController(motorID);

      case SPARK_MAX_BRUSHED:
        return new SparkMaxController(motorID, CANSparkMaxLowLevel.MotorType.kBrushed);

      case SPARK_MAX_BRUSHLESS:
        return new SparkMaxController(motorID, CANSparkMaxLowLevel.MotorType.kBrushless);

      case NONE:
      default:
        throw new IllegalArgumentException("Illegal motor type " + type.name());
    }
  }
}
