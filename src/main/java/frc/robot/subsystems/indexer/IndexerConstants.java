package frc.robot.subsystems.indexer;

import frc.lib.utils.TunableNumber;
import frc.robot.Constants;

public class IndexerConstants {
  public static final double INDEXER_FOWARD_VOLTAGE;
  public static final double INDEXER_HOLD_VOLTAGE;
  public static final double INDEXER_BACKWARD_VOLTAGE;
  public static final int INDEXER_LIMIT_SWITCH_ID;
  public static final double WHEEL_DIAMETER;
  public static final TunableNumber INDEX_SPEED;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_SIMBOT -> {
        INDEXER_FOWARD_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = 0.0;
        INDEXER_LIMIT_SWITCH_ID = 0;
        WHEEL_DIAMETER = 4;
        INDEX_SPEED = new TunableNumber("Indexer/IndexSpeed");
      }
      case ROBOT_2024C -> {
        INDEXER_FOWARD_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = 0.0;
        INDEXER_LIMIT_SWITCH_ID = 13;
        WHEEL_DIAMETER = 4;
        INDEX_SPEED = new TunableNumber("Indexer/IndexSpeed", 0.6);
      }
      default -> {
        INDEXER_FOWARD_VOLTAGE = 0.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = 0.0;
        WHEEL_DIAMETER = 0.0;
        INDEXER_LIMIT_SWITCH_ID = 0;
        INDEX_SPEED = new TunableNumber("Indexer/IndexSpeed");
      }
    }
  }
}
