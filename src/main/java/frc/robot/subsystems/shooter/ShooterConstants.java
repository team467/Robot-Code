package frc.robot.subsystems.shooter;

import frc.robot.Constants;

public class ShooterConstants {
  public static final double INDEXER_FOWARD_VOLTAGE;
  public static final double INDEXER_HOLD_VOLTAGE;
  public static final double INDEXER_BACKWARD_VOLTAGE;
  public static final double SHOOTER_READY_VELOCITY_RAD_PER_SEC;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023 -> {
        INDEXER_FOWARD_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = -3.0;
        SHOOTER_READY_VELOCITY_RAD_PER_SEC = 0.6;
      }
      case ROBOT_SIMBOT -> {
        INDEXER_FOWARD_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = 0.0;
        SHOOTER_READY_VELOCITY_RAD_PER_SEC = 0.0;
      }
      default -> {
        INDEXER_FOWARD_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = 0.0;
        SHOOTER_READY_VELOCITY_RAD_PER_SEC = 0.0;
      }
    }
  }
}
