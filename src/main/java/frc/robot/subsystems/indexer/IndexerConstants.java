// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import frc.robot.Constants;

/** Add your docs here. */
public class IndexerConstants {
  public static final double INDEXER_FOWARD_VOLTAGE;
  public static final double INDEXER_MAX_VOLTAGE;
  public static final double INDEXER_HOLD_VOLTAGE;
  public static final double INDEXER_BACKWARD_VOLTAGE;
  public static final int INDEXER_ID;
  public static final int INDEXER_LIMIT_SWITCH_ID;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_BRIEFCASE -> {
        INDEXER_FOWARD_VOLTAGE = INDEXER_MAX_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = -3.0;
        INDEXER_ID = 2;
        INDEXER_LIMIT_SWITCH_ID = 0;
      }
      case ROBOT_SIMBOT -> {
        INDEXER_FOWARD_VOLTAGE = INDEXER_MAX_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = 0.0;
        INDEXER_ID = 0;
        INDEXER_LIMIT_SWITCH_ID = 0;
      }
      default -> {
        INDEXER_FOWARD_VOLTAGE = INDEXER_MAX_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = 0.0;
        INDEXER_ID = 0;
        INDEXER_LIMIT_SWITCH_ID = 0;
      }
    }
  }
}
