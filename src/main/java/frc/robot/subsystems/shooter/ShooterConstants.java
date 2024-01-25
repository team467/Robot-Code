package frc.robot.subsystems.shooter;

import frc.lib.utils.TunableNumber;
import frc.robot.Constants;

public class ShooterConstants {
  public static final double INDEXER_FOWARD_VOLTAGE;
  public static final double INDEXER_HOLD_VOLTAGE;
  public static final double INDEXER_BACKWARD_VOLTAGE;
  public static final double SHOOTER_READY_VELOCITY_RAD_PER_SEC;
  public static final TunableNumber SHOOTER_KS;
  public static final TunableNumber SHOOTER_KV;
  public static final int SHOOTER_LEADER_ID;
  public static final int SHOOTER_FOLLOWER_ID;
  public static final int INDEXER_ID;
  public static final double SHOOTER_KP;
  public static final double SHOOTER_KD;
  static {
    switch (Constants.getRobot()) {
      case ROBOT_2024A -> {
        INDEXER_FOWARD_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = -3.0;
        SHOOTER_READY_VELOCITY_RAD_PER_SEC = 0.6;
        SHOOTER_KS = new TunableNumber("Shooter/Module/ShooterKS", 0.49385);
        SHOOTER_KV = new TunableNumber("Shooter/Module/ShooterKV", 2.60818);
        SHOOTER_LEADER_ID = 1;
        SHOOTER_FOLLOWER_ID = 2;
        INDEXER_ID = 3;
        SHOOTER_KP = 0.0;
        SHOOTER_KD = 0.0;
      }
      case ROBOT_SIMBOT -> {
        INDEXER_FOWARD_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = 0.0;
        SHOOTER_READY_VELOCITY_RAD_PER_SEC = 0.0;
        SHOOTER_KS = new TunableNumber("Shooter/Module/ShooterKS", 0.49385);
        SHOOTER_KV = new TunableNumber("Shooter/Module/ShooterKV", 2.60818);
        SHOOTER_LEADER_ID = 0;
        SHOOTER_FOLLOWER_ID = 0;
        INDEXER_ID = 0;
        SHOOTER_KP = 0.0;
        SHOOTER_KD = 0.0;
      }
      default -> {
        INDEXER_FOWARD_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = 0.0;
        SHOOTER_READY_VELOCITY_RAD_PER_SEC = 0.0;
        SHOOTER_KS = new TunableNumber("Shooter/Module/ShooterKS", 0.49385);
        SHOOTER_KV = new TunableNumber("Shooter/Module/ShooterKV", 2.60818);
        SHOOTER_LEADER_ID = 0;
        SHOOTER_FOLLOWER_ID = 0;
        INDEXER_ID = 0;
        SHOOTER_KP = 0.0;
        SHOOTER_KD = 0.0;
      }
    }
  }
}
