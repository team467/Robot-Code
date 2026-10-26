package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.RPM;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.AngularVelocity;

public class ShooterConstants {

  public static final double VOLTAGE_COMPENSATION = 12.0;
  public static final int CURRENT_LIMIT = 35;
  public static final IdleMode IDLE_MODE = IdleMode.kCoast;

  public static final double KV = 0.0662;
  public static final double KA = 0.028853;
  public static final double KS = 0.2723;

  public static final double SHOOTER_WHEEL_GEAR_RATIO = 3.5;
  public static final AngularVelocity CLOSE_HUB_SHOOTER_RPM = RPM.of(1100);
  public static final double MAX_VOLTAGE = 12.0;

  public static final double TOLERANCE =
      Units.rotationsPerMinuteToRadiansPerSecond(100); // measured in radians

  public static Transform2d kShooterOffsetFromRobotCenter =
      new Transform2d(new Translation2d(-0.1585, 0.0), new Rotation2d(0.0));
}
