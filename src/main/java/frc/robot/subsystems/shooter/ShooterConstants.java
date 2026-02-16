package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ShooterConstants {
  public static final double ENCODER_POSITION_CONVERSION = (2 * Math.PI / 60.0);
  public static final double ENCODER_VELOCITY_CONVERSION = (2 * Math.PI / 60.0);
  //
  //  public static final double ENCODER_POSITION_CONVERSION = 1.0;
  //  public static final double ENCODER_VELOCITY_CONVERSION = 1.0;

  public static final double VOLTAGE_COMPENSATION = 12.0;
  public static final int CURRENT_LIMIT = 30;
  public static final IdleMode IDLE_MODE = IdleMode.kCoast;

  //  public static final double KV = 0.002027;  //single motor
  //  public static final double KA = 0.00033377;  //single motor

  public static final double KV = 0.0019641; // casebot with flywheel
  public static final double KA = 0.00017327; // casebot with flywheel

  public static final double FLYWHEEL_GEAR_RATIO = 2.5;

  public static final double MAX_VOLTAGE = 12.0;
}
