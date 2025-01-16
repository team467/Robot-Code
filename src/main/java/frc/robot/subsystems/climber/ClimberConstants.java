package frc.robot.subsystems.climber;

// import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimberConstants {
  public static final int CLIMBER_RATCHET_ID;
  public static final double ROTS_TO_METERS;
  public static final double CLIMBER_FORWARD_PERCENT;
  public static final double CLIMBER_BACKWARD_PERCENT;
  public static final double BACKUP_TIME;
  public static final float LOWER_LIMIT_POSITION;
  public static final float UPPER_LIMIT_POSITION;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2025_COMP -> {
        CLIMBER_RATCHET_ID = 0;
        ROTS_TO_METERS = 0.0;
        CLIMBER_FORWARD_PERCENT = 0.0;
        CLIMBER_BACKWARD_PERCENT = 0.0;
        BACKUP_TIME = 0.0;
        LOWER_LIMIT_POSITION = 0.0f;
        UPPER_LIMIT_POSITION = 0.0f;
      }
      default -> {
        CLIMBER_RATCHET_ID = 0;
        ROTS_TO_METERS = 0.0;
        CLIMBER_FORWARD_PERCENT = 0.0;
        CLIMBER_BACKWARD_PERCENT = 0.0;
        BACKUP_TIME = 0.0;
        LOWER_LIMIT_POSITION = 0.0f;
        UPPER_LIMIT_POSITION = 0.0f;
      }
    }
  }
}
