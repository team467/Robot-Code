package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

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
  public static final double CLOSE_HUB_SHOOTER_RPM = 1085;

  public static final double MAX_VOLTAGE = 12.0;

  public static final double TOLERANCE = 50;

  public static Transform2d kShooterOffsetFromRobotCenter =
      new Transform2d(new Translation2d(-0.163, 0.0), new Rotation2d(0.0));

  public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap shooterRpmMap = new InterpolatingDoubleTreeMap();

  static {
    timeOfFlightMap.put(1.50, 0.74);
    timeOfFlightMap.put(1.90, 0.93);
    timeOfFlightMap.put(2.23, 1.02);
    timeOfFlightMap.put(2.39, 1.14);
    timeOfFlightMap.put(2.55, 1.21);
    timeOfFlightMap.put(2.70, 1.26);
    timeOfFlightMap.put(2.82, 1.22);
    timeOfFlightMap.put(3.17, 1.24);
    timeOfFlightMap.put(3.36, 1.35);
    timeOfFlightMap.put(4.02, 1.39);

    shooterRpmMap.put(1.50, 1065.0);
    shooterRpmMap.put(1.90, 1145.0);
    shooterRpmMap.put(2.23, 1200.0);
    shooterRpmMap.put(2.39, 1245.0);
    shooterRpmMap.put(2.55, 1300.0);
    shooterRpmMap.put(2.70, 1350.0);
    shooterRpmMap.put(2.82, 1370.0);
    shooterRpmMap.put(3.17, 1380.0);
    shooterRpmMap.put(3.36, 1410.0);
    shooterRpmMap.put(4.02, 1520.0);
  }
}
