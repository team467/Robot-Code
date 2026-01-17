package frc.robot.subsystems.indexer;

import frc.robot.Constants;

public class IndexConstants {
  public static final double ENCODER_POSITION_CONVERSION;
  public static final double ENCODER_VELOCITY_CONVERSION;

  public static final int INDEXER_INDEXERMOTOR_ID;
  public static final int INDEXER_FEEDUP_ID;

  public static final int INDEX_PERCENT;
  public static final int FEEDUP_PERCENT;
  public static final int INDEX_VOLT;
  public static final int FEEDUP_VOLT;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2025_COMP -> {
        ENCODER_POSITION_CONVERSION = 0;
        ENCODER_VELOCITY_CONVERSION = 0;

        INDEX_VOLT = 0;
        FEEDUP_VOLT = 0;

        INDEX_PERCENT = 0;
        FEEDUP_PERCENT = 0;

        INDEXER_FEEDUP_ID = 0;
        INDEXER_INDEXERMOTOR_ID = 0;
      }
      case ROBOT_BRIEFCASE -> {
        ENCODER_POSITION_CONVERSION = 0;
        ENCODER_VELOCITY_CONVERSION = 0;
        INDEXER_FEEDUP_ID = 0;
        INDEXER_INDEXERMOTOR_ID = 0;

        INDEX_VOLT = 0;
        FEEDUP_VOLT = 0;

        INDEX_PERCENT = 0;
        FEEDUP_PERCENT = 0;
      }
      default -> {
        ENCODER_POSITION_CONVERSION = 0;
        ENCODER_VELOCITY_CONVERSION = 0;
        INDEXER_FEEDUP_ID = 0;
        INDEXER_INDEXERMOTOR_ID = 0;

        INDEX_VOLT = 0;
        FEEDUP_VOLT = 0;

        INDEX_PERCENT = 0;
        FEEDUP_PERCENT = 0;
      }
    }
  }
}
