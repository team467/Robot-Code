package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.RPM;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.lib.utils.MakeTunable;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.AngularVelocity;

@MakeTunable
public class ShooterConstants {

  public static double VOLTAGE_COMPENSATION = 12.0;
  public static int CURRENT_LIMIT = 35;
  public static IdleMode IDLE_MODE = IdleMode.kCoast;

  public static double KV = 0.0662;
  public static double KA = 0.028853;
  public static double KS = 0.2723;

  public static AngularVelocity CLOSE_HUB_SHOOTER_RPM = RPM.of(1100);
  public static double GEAR_RATIO = 3.5;

  public static Transform2d kShooterOffsetFromRobotCenter =
      new Transform2d(new Translation2d(-0.1585, 0.0), new Rotation2d(0.0));
}
