package frc.robot.subsystems.shooter;

import frc.lib.utils.TunableNumber;
import frc.robot.Constants;

public class ShooterConstants {
  public static final double INDEXER_FOWARD_VOLTAGE;
  public static final double INDEXER_HOLD_VOLTAGE;
  public static final double INDEXER_BACKWARD_VOLTAGE;
  public static final double FLYWHEEL_READY_VELOCITY_RAD_PER_SEC;
  public static final TunableNumber FLYWHEEL_KS;
  public static final TunableNumber FLYWHEEL_KV;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023 -> {
        INDEXER_FOWARD_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = -3.0;
        FLYWHEEL_READY_VELOCITY_RAD_PER_SEC = 0.6;
        FLYWHEEL_KS = new TunableNumber("Shooter/Module/ShooterKS", 0.49385);
        FLYWHEEL_KV = new TunableNumber("Shooter/Module/ShooterKV", 2.60818);
      }
      case ROBOT_SIMBOT -> {
        INDEXER_FOWARD_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = 0.0;
        FLYWHEEL_READY_VELOCITY_RAD_PER_SEC = 0.0;
        FLYWHEEL_KS = new TunableNumber("Shooter/Module/ShooterKS", 0.49385);
        FLYWHEEL_KV = new TunableNumber("Shooter/Module/ShooterKV", 2.60818);
      }
      default -> {
        INDEXER_FOWARD_VOLTAGE = 5.0;
        INDEXER_HOLD_VOLTAGE = 0.0;
        INDEXER_BACKWARD_VOLTAGE = 0.0;
        FLYWHEEL_READY_VELOCITY_RAD_PER_SEC = 0.0;
        FLYWHEEL_KS = new TunableNumber("Shooter/Module/ShooterKS", 0.49385);
        FLYWHEEL_KV = new TunableNumber("Shooter/Module/ShooterKV", 2.60818);
      }
    }
  }
}
