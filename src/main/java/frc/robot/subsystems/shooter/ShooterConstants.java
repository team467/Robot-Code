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
  public static final double CLOSE_HUB_SHOOTER_RPM = 1085;

  public static final double MAX_VOLTAGE = 12.0;

  public static final double TOLERANCE = 50; // measured in radians

  public static Transform2d kShooterOffsetFromRobotCenter =
      new Transform2d(new Translation2d(-0.163, 0.0), new Rotation2d(0.0));

  // ── Fire control / projectile sim parameters ──
  // TODO: Measure these from your CAD model and update:

  // Launcher position relative to robot center (meters)
  // Negative X = behind center, positive X = in front
  // Your existing offset says 0.163m behind center
  public static final double LAUNCHER_OFFSET_X = -0.163; // TODO: measure from CAD (meters forward)
  public static final double LAUNCHER_OFFSET_Y = 0.0; // TODO: measure from CAD (meters left)

  // Launcher exit height from floor (meters) — measure from CAD
  public static final double EXIT_HEIGHT_M = 0.43; // TODO: measure from CAD

  // Fixed launch angle from horizontal (degrees) — measure from CAD
  public static final double LAUNCH_ANGLE_DEG = 45.0; // TODO: measure from CAD

  // Shooter wheel diameter (meters) — measure with calipers on the actual wheel
  public static final double WHEEL_DIAMETER_M = 0.1016; // TODO: measure with calipers (4" default)

  // Slip factor: how much ball speed vs wheel surface speed (0 = no grip, 1 = perfect grip)
  // Start at 0.6 and tune on robot by comparing actual vs predicted shot distance
  public static final double SLIP_FACTOR = 0.6; // TODO: tune on robot

  // Ball properties (from 2026 REBUILT game manual)
  public static final double BALL_MASS_KG = 0.215;
  public static final double BALL_DIAMETER_M = 0.1501;

  // Target height (meters) — hub inner opening height from game manual
  public static final double TARGET_HEIGHT_M = 1.83; // TODO: verify from game manual

  // Aerodynamic coefficients (standard values, shouldn't need changing)
  public static final double DRAG_COEFF = 0.47;
  public static final double MAGNUS_COEFF = 0.2;
  public static final double AIR_DENSITY = 1.225;

  // ShotCalculator config
  public static final double PHASE_DELAY_MS = 30.0; // vision pipeline latency
  public static final double MECH_LATENCY_MS = 20.0; // mechanism response time

  // Projectile sim parameters
  public static final ProjectileSimulator.SimParameters SIM_PARAMS =
      new ProjectileSimulator.SimParameters(
          BALL_MASS_KG,
          BALL_DIAMETER_M,
          DRAG_COEFF,
          MAGNUS_COEFF,
          AIR_DENSITY,
          EXIT_HEIGHT_M,
          WHEEL_DIAMETER_M,
          TARGET_HEIGHT_M,
          SLIP_FACTOR,
          LAUNCH_ANGLE_DEG,
          0.001, // sim timestep
          1500, // RPM min search
          6000, // RPM max search
          25, // binary search iterations
          5.0 // max sim time
          );
}
