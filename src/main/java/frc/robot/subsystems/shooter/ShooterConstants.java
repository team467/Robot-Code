package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import frc.lib.utils.TunableNumber;
import frc.robot.Constants;

public class ShooterConstants {
  public static final double SHOOTER_READY_VELOCITY_RAD_PER_SEC;
  public static final TunableNumber SHOOTER_KS;
  public static final TunableNumber SHOOTER_KV;
  public static final int SHOOTER_LEADER_ID;
  public static final int SHOOTER_FOLLOWER_ID;
  public static final int SHOOTER_2_ID;
  public static final double WHEEL_DIAMETER;
  public static final TunableNumber SHOOTER_KP;
  public static final TunableNumber SHOOTER_KD;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_BRIEFCASE -> {
        // TODO: SHOOTER FEEDFOWARD NEEDS TO BE TUNED DUE TO A UNIT CONVERSION
        // TODO: SHOOTER PID NEEDS TO BE TUNED
        WHEEL_DIAMETER = Units.inchesToMeters(4);
        SHOOTER_READY_VELOCITY_RAD_PER_SEC = 1000;
        SHOOTER_KS = new TunableNumber("Shooter/ShooterKS", 0.8);
        SHOOTER_KV = new TunableNumber("Shooter/ShooterKV", 0.0046);
        SHOOTER_LEADER_ID = 1;
        SHOOTER_FOLLOWER_ID = 2;
        SHOOTER_2_ID = 1;
        SHOOTER_KP = new TunableNumber("Shooter/ShooterKP", 0.0);
        SHOOTER_KD = new TunableNumber("Shooter/ShooterKD", 0.0);
      }
      case ROBOT_SIMBOT -> {
        WHEEL_DIAMETER = 0.0;
        SHOOTER_READY_VELOCITY_RAD_PER_SEC = 0.0;
        SHOOTER_KS = new TunableNumber("Shooter/ShooterKS", 0.0);
        SHOOTER_KV = new TunableNumber("Shooter/ShooterKV", 0.0);
        SHOOTER_LEADER_ID = 0;
        SHOOTER_FOLLOWER_ID = 0;
        SHOOTER_2_ID = 3;
        SHOOTER_KP = new TunableNumber("Shooter/ShooterKP", 0.0);
        SHOOTER_KD = new TunableNumber("Shooter/ShooterKD", 0.0);
      }
      default -> {
        WHEEL_DIAMETER = 0.0;
        SHOOTER_READY_VELOCITY_RAD_PER_SEC = 0.0;
        SHOOTER_KS = new TunableNumber("Shooter/ShooterKS", 0.0);
        SHOOTER_KV = new TunableNumber("Shooter/ShooterKV", 0.0);
        SHOOTER_LEADER_ID = 0;
        SHOOTER_FOLLOWER_ID = 0;
        SHOOTER_2_ID = 3;
        SHOOTER_KP = new TunableNumber("Shooter/ShooterKP", 0.0);
        SHOOTER_KD = new TunableNumber("Shooter/ShooterKD", 0.0);
      }
    }
  }
}
