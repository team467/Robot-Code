package frc.robot.subsystems.shooter;

import frc.robot.Constants;

public class ShooterConstants {
  public static double indexerFowardVoltage;
  public static double indexerHoldVoltage;
  public static double indexerBackwardVolatage;
  public static double shooterReadyVelocity;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023 -> {
        indexerFowardVoltage = 5.0;
        indexerHoldVoltage = 0.0;
        indexerBackwardVolatage = -3.0;
        shooterReadyVelocity = 0.6;
      }
      case ROBOT_SIMBOT -> {
        indexerFowardVoltage = 0.0;
        indexerHoldVoltage = 0.0;
        indexerBackwardVolatage = 0.0;
        shooterReadyVelocity = 0.0;
        
      }
      default -> {
        indexerFowardVoltage = 0.0;
        indexerHoldVoltage = 0.0;
        indexerBackwardVolatage = 0.0;
        shooterReadyVelocity = 0.0;
      }
    }
  }
}
