package frc.robot.subsystems.climber;

import frc.robot.Constants;

public class ClimberConstants {
  public static final int CLIMBER_RATCHET_ID;
  public static final double ROTS_TO_METERS;
  public static final double CLIMBER_FORWARD_PERCENT;
  public static final double CLIMBER_BACKWARD_PERCENT;
  public static final double BACKUP_TIME;
  public static final float SOFT_LIMIT_POSITION;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2024_COMP -> {
        CLIMBER_RATCHET_ID = 0;
        ROTS_TO_METERS = 1.0;
        CLIMBER_FORWARD_PERCENT = 0.6;
        CLIMBER_BACKWARD_PERCENT = -0.6;
        BACKUP_TIME = 0.1;
        SOFT_LIMIT_POSITION = 0.1f;
      }
      default -> {
        CLIMBER_RATCHET_ID = 0;
        ROTS_TO_METERS = 0.0;
        CLIMBER_FORWARD_PERCENT = 0.0;
        CLIMBER_BACKWARD_PERCENT = 0.0;
        BACKUP_TIME = 0.0;
        SOFT_LIMIT_POSITION = 0.0f;
      }
    }
  }
}
