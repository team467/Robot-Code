package frc.robot.subsystems.shooter;

import frc.robot.Constants;

public class ShooterConstants {
  public static double indexerFowardVoltage;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023 -> {
        indexerFowardVoltage = 5.0;
      }
      case ROBOT_SIMBOT -> {
        indexerFowardVoltage = 0.0;
      }
      default -> {
        indexerFowardVoltage = 0.0;
      }
    }
  }
}
