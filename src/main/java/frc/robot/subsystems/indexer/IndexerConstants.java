package frc.robot.subsystems.indexer;

import frc.lib.utils.TunableNumber;
import frc.robot.Constants;
import frc.robot.constants.controls.GearRatio;

public class IndexerConstants {
  public static final double INDEXER_FOWARD_VOLTAGE;
  public static final double INDEXER_HOLD_VOLTAGE;
  public static final double INDEXER_BACKWARD_VOLTAGE;
  public static final int INDEXER_LIMIT_SWITCH_ID;
  public static final double WHEEL_DIAMETER;
  public static final TunableNumber INDEX_SPEED;
  public static final GearRatio INDEXER_GEAR_RATIO;
  public static final double BACKUP_SPEED;
  public static final double BACKUP_TIME;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_SIMBOT -> {
        INDEXER_FOWARD_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = 0.0;
        INDEXER_LIMIT_SWITCH_ID = 0;
        WHEEL_DIAMETER = 4;
        INDEX_SPEED = new TunableNumber("Indexer/IndexSpeed");
        INDEXER_GEAR_RATIO = new GearRatio();
        BACKUP_SPEED = 0.0;
        BACKUP_TIME = 0.0;
      }
      case ROBOT_2024_COMP -> {
        INDEXER_FOWARD_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = 0.0;
        INDEXER_LIMIT_SWITCH_ID = 13;
        WHEEL_DIAMETER = 4;
        INDEX_SPEED = new TunableNumber("Indexer/IndexSpeed", 1);
        INDEXER_GEAR_RATIO = new GearRatio(1.5, 1);
        BACKUP_SPEED = -0.3;
        BACKUP_TIME = 0.3;
      }
      default -> {
        INDEXER_FOWARD_VOLTAGE = 0.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = 0.0;
        WHEEL_DIAMETER = 0.0;
        INDEXER_LIMIT_SWITCH_ID = 0;
        INDEX_SPEED = new TunableNumber("Indexer/IndexSpeed");
        INDEXER_GEAR_RATIO = new GearRatio();
        BACKUP_SPEED = 0.0;
        BACKUP_TIME = 0.0;
      }
    }
  }
}
