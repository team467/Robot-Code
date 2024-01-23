package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;

public class Schematic {
  // Power Distribution Module
  public static final int POWER_DIST_ID;
  public static final PowerDistribution.ModuleType POWER_DIST_TYPE;
  // Drive
  public static final int REAR_RIGHT_DRIVE_ID;
  public static final int REAR_RIGHT_STEERING_ID;
  public static final int REAR_LEFT_DRIVE_ID;
  public static final int REAR_LEFT_STEERING_ID;
  public static final int FRONT_LEFT_DRIVE_ID;
  public static final int FRONT_LEFT_STEERING_ID;
  public static final int FRONT_RIGHT_DRIVE_ID;
  public static final int FRONT_RIGHT_STEERING_ID;
  public static final int REAR_RIGHT_CANCODER_ID;
  public static final int REAR_LEFT_CANCODER_ID;
  public static final int FRONT_LEFT_CANCODER_ID;
  public static final int FRONT_RIGHT_CANCODER_ID;
  public static final int GYRO_ID;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023 -> {
        POWER_DIST_ID = 20;
        POWER_DIST_TYPE = PowerDistribution.ModuleType.kRev;

        FRONT_LEFT_DRIVE_ID = 3;
        FRONT_LEFT_STEERING_ID = 4;
        FRONT_LEFT_CANCODER_ID = 13;
        FRONT_RIGHT_DRIVE_ID = 5;
        FRONT_RIGHT_STEERING_ID = 6;
        FRONT_RIGHT_CANCODER_ID = 14;
        REAR_LEFT_DRIVE_ID = 1;
        REAR_LEFT_STEERING_ID = 2;
        REAR_LEFT_CANCODER_ID = 15;
        REAR_RIGHT_DRIVE_ID = 7;
        REAR_RIGHT_STEERING_ID = 8;
        REAR_RIGHT_CANCODER_ID = 16;
        GYRO_ID = 17;
      }
      case ROBOT_2024A -> {
        POWER_DIST_ID = 20;
        POWER_DIST_TYPE = PowerDistribution.ModuleType.kCTRE;

        REAR_RIGHT_DRIVE_ID = 1;
        REAR_RIGHT_STEERING_ID = 2;
        REAR_RIGHT_CANCODER_ID = 13;
        REAR_LEFT_DRIVE_ID = 3;
        REAR_LEFT_STEERING_ID = 4;
        REAR_LEFT_CANCODER_ID = 14;
        FRONT_LEFT_DRIVE_ID = 5;
        FRONT_LEFT_STEERING_ID = 6;
        FRONT_LEFT_CANCODER_ID = 15;
        FRONT_RIGHT_DRIVE_ID = 7;
        FRONT_RIGHT_STEERING_ID = 8;
        FRONT_RIGHT_CANCODER_ID = 16;
        GYRO_ID = 0;
      }
      default -> {
        POWER_DIST_ID = 20;
        POWER_DIST_TYPE = PowerDistribution.ModuleType.kAutomatic;

        REAR_RIGHT_DRIVE_ID = 0;
        REAR_RIGHT_STEERING_ID = 0;
        REAR_RIGHT_CANCODER_ID = 0;
        REAR_LEFT_DRIVE_ID = 0;
        REAR_LEFT_STEERING_ID = 0;
        REAR_LEFT_CANCODER_ID = 0;
        FRONT_LEFT_DRIVE_ID = 0;
        FRONT_LEFT_STEERING_ID = 0;
        FRONT_LEFT_CANCODER_ID = 0;
        FRONT_RIGHT_DRIVE_ID = 0;
        FRONT_RIGHT_STEERING_ID = 0;
        FRONT_RIGHT_CANCODER_ID = 0;
        GYRO_ID = 0;
      }
    }
  }
}
