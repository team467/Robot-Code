package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ShooterConstants {
  public static final double ENCODER_POSITION_CONVERSION = 1.0;
  public static final double ENCODER_VELOCITY_CONVERSION = 1.0;

  public static final double VOLTAGE_COMPENSATION = 12.0;
  public static final int CURRENT_LIMIT = 30;
  public static final IdleMode IDLE_MODE = IdleMode.kCoast;

  public static final double PID_P = 0.00007;
  public static final double PID_I = 0.0000003;
  public static final double PID_D = 0.00000;
}
