package first.robot.mechanisms.shooter;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import first.robot.Schematic;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

public final class ShooterConstants {
  public static final CANPort CAN_PORT = Schematic.canPort;

  public static final double ENCODER_POSITION_CONVERSION = 1;
  public static final double ENCODER_VELOCITY_CONVERSION = (2 * Math.PI) / 60;

  public static final double ACONST = 25.30184;
  public static final double BCONST = -65.12642;
  public static final double CCONST = 1001.70713;
  public static final double MAX_SETPOINT_RPM = 1550;

  public static final double VOLTAGE_COMPENSATION = 12.0;
  public static final int CURRENT_LIMIT = 35;
  public static final IdleMode IDLE_MODE = IdleMode.kCoast;

  public static final double KV = 0.0662;
  public static final double KA = 0.028853;
  public static final double KS = 0.2723;

  public static final double FF_SCALAR = 1;

  public static final double SHOOTER_WHEEL_GEAR_RATIO = 3.5;

  public static final double CLOSE_HUB_SHOOTER_RPM = 1100;
  public static final double MAX_VOLTAGE = 12.0;

  public static final double TARGET_RAMP_RATE = 800;

  public static final double TOLERANCE =
      Units.rotationsPerMinuteToRadiansPerSecond(100);

  public static final double KP = 0.05;

  public static final double READY_MIN_FRACTION = 0.85;
  public static final double SETTLED_ACCEL = 20.0;
  public static final double SETTLED_TIME = 0.2;

  public static final double SATURATION_MARGIN_VOLTS = 0.5;

  public static final Transform2d kShooterOffsetFromRobotCenter =
      new Transform2d(new Translation2d(-0.1585, 0.0), new Rotation2d(0.0));

  private ShooterConstants() {}
}
