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
  public static final int SHOOTER_RIGHT_ID;
  public static final int SHOOTER_LEFT_ID;
  public static final int INDEXER_ID;
  // Intake
  public static final int INTAKE_ID;
  public static final int ARM_ID_LEADER;
  public static final int ARM_ID_FOLLOWER;
  public static final int CLIMBER_LEFT_ID;
  public static final int CLIMBER_RIGHT_ID;

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
        SHOOTER_RIGHT_ID = 0;
        SHOOTER_LEFT_ID = 0;
        INDEXER_ID = 0;

        INTAKE_ID = 0;
        ARM_ID_LEADER = 0;
        ARM_ID_FOLLOWER = 0;
        CLIMBER_RIGHT_ID = 0;
        CLIMBER_LEFT_ID = 0;
      }
      case ROBOT_2024C -> {
        POWER_DIST_ID = 20;
        POWER_DIST_TYPE = PowerDistribution.ModuleType.kCTRE;
        REAR_RIGHT_DRIVE_ID = 7;
        REAR_RIGHT_STEERING_ID = 8;
        REAR_RIGHT_CANCODER_ID = 19;
        REAR_LEFT_DRIVE_ID = 5;
        REAR_LEFT_STEERING_ID = 6;
        REAR_LEFT_CANCODER_ID = 20;
        FRONT_LEFT_DRIVE_ID = 1;
        FRONT_LEFT_STEERING_ID = 2;
        FRONT_LEFT_CANCODER_ID = 21;
        FRONT_RIGHT_DRIVE_ID = 3;
        FRONT_RIGHT_STEERING_ID = 4;
        FRONT_RIGHT_CANCODER_ID = 18;
        GYRO_ID = 17;
        SHOOTER_RIGHT_ID = 11; // TODO: Confirm
        SHOOTER_LEFT_ID = 12;
        INDEXER_ID = 13;
        INTAKE_ID = 14;
        ARM_ID_LEADER = 9;
        ARM_ID_FOLLOWER = 10;
        CLIMBER_RIGHT_ID = 15;
        CLIMBER_LEFT_ID = 16;
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
        SHOOTER_RIGHT_ID = 0;
        SHOOTER_LEFT_ID = 0;
        INDEXER_ID = 0;

        INTAKE_ID = 0;
        ARM_ID_LEADER = 0;
        ARM_ID_FOLLOWER = 0;
        CLIMBER_RIGHT_ID = 0;
        CLIMBER_LEFT_ID = 0;
      }
    }
  }
}
