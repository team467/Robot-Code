package frc.robot.subsystems.indexer;

import frc.robot.Constants;

public class IndexConstants {
  public static final double ENCODER_POSITION_CONVERSION;
  public static final double ENCODER_VELOCITY_CONVERSION;

  public static final int INDEXER_INTAKEMOTOR_ID;
  public static final int INDEXER_FEEDUP_ID;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2025_COMP -> {
        ENCODER_POSITION_CONVERSION = 0;
        ENCODER_VELOCITY_CONVERSION = 0;

        INDEXER_FEEDUP_ID = 0;
        INDEXER_INTAKEMOTOR_ID = 0;
      }
      case ROBOT_BRIEFCASE -> {
        ENCODER_POSITION_CONVERSION = 0;
        ENCODER_VELOCITY_CONVERSION = 0;
        INDEXER_FEEDUP_ID = 0;
        INDEXER_INTAKEMOTOR_ID = 0;
      }
      default -> {
        ENCODER_POSITION_CONVERSION = 0;
        ENCODER_VELOCITY_CONVERSION = 0;
        INDEXER_FEEDUP_ID = 0;
        INDEXER_INTAKEMOTOR_ID = 0;
      }
    }
  }
}
