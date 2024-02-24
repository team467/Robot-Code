package frc.robot.subsystems.shooter;

import frc.lib.utils.TunableNumber;
import frc.robot.Constants;

public class ShooterConstants {
  public static final double SHOOTER_READY_VELOCITY_RAD_PER_SEC;
  public static final TunableNumber SHOOTER_KS;
  public static final TunableNumber SHOOTER_KV;
  public static final TunableNumber SHOOTER_KP;
  public static final TunableNumber SHOOTER_KD;
  public static final TunableNumber SHOOTER_TOLERANCE;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_SIMBOT -> {
        SHOOTER_READY_VELOCITY_RAD_PER_SEC = 0.0;
        SHOOTER_KS = new TunableNumber("Shooter/ShooterKS", 0.0);
        SHOOTER_KV = new TunableNumber("Shooter/ShooterKV", 0.0);
        SHOOTER_KP = new TunableNumber("Shooter/ShooterKP", 0.0);
        SHOOTER_KD = new TunableNumber("Shooter/ShooterKD", 0.0);
        SHOOTER_TOLERANCE = new TunableNumber("Shooter/ShooterTolerance", 0.0);
      }
      case ROBOT_2024C -> {
        SHOOTER_READY_VELOCITY_RAD_PER_SEC = 0.0;
        SHOOTER_KS = new TunableNumber("Shooter/ShooterKS", 0.05);
        SHOOTER_KV = new TunableNumber("Shooter/ShooterKV", 0.05);
        SHOOTER_KP = new TunableNumber("Shooter/ShooterKP", 1.0);
        SHOOTER_KD = new TunableNumber("Shooter/ShooterKD", 0.01);
        SHOOTER_TOLERANCE = new TunableNumber("Shooter/ShooterTolerance", 0.1);
      }
      default -> {
        SHOOTER_READY_VELOCITY_RAD_PER_SEC = 0.0;
        SHOOTER_KS = new TunableNumber("Shooter/ShooterKS", 0.0);
        SHOOTER_KV = new TunableNumber("Shooter/ShooterKV", 0.0);
        SHOOTER_KP = new TunableNumber("Shooter/ShooterKP", 0.0);
        SHOOTER_KD = new TunableNumber("Shooter/ShooterKD", 0.0);
        SHOOTER_TOLERANCE = new TunableNumber("Shooter/ShooterTolerance", 0.0);
      }
    }
  }
}
