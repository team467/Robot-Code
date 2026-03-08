package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShooterConstants {
  public static final double ENCODER_POSITION_CONVERSION = 1;
  public static final double ENCODER_VELOCITY_CONVERSION = (2 * Math.PI) / 60;

  public static final double VOLTAGE_COMPENSATION = 12.0;
  public static final int CURRENT_LIMIT = 20;
  public static final IdleMode IDLE_MODE = IdleMode.kCoast;

  public static final double KV = 0.0465;
  public static final double KA = 0.031259;
  public static final double KS = 0.25;

  public static final double SHOOTER_WHEEL_GEAR_RATIO = 2.5;

  public static final double MAX_VOLTAGE = 12.0;

  public static final double TOLERANCE = 50; // measured in radians

  public static Transform2d kShooterOffsetFromRobotCenter =
      new Transform2d(new Translation2d(-0.163, 0.0), new Rotation2d(0.0));
}
