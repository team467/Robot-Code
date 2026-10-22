package first.robot.mechanisms.swerve;

import static java.lang.Math.PI;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;

public class DriveConstants {
  public static final double odometryFrequency = 100.0;
  public static final double trackWidth = 0.59055;
  public static final double wheelBase = 0.5207;
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  public static final Rotation2d frontLeftZeroRotation = Rotation2d.fromRadians(0.126 + PI);
  public static final Rotation2d frontRightZeroRotation = Rotation2d.fromRadians(0.759 + 0.03);
  public static final Rotation2d backLeftZeroRotation = Rotation2d.fromRadians(-1.630);
  public static final Rotation2d backRightZeroRotation = Rotation2d.fromRadians(-2.072);

  public static final SwerveModuleConstants.ClosedLoopOutputType driveClosedLoopOutput =
      ClosedLoopOutputType.Voltage;
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.98);
  public static final double driveMotorReduction = 8.16;
  public static final DCMotor driveGearbox = DCMotor.getKrakenX60Foc(1);

  public static final double maxSpeedMetersPerSec =
      driveGearbox.freeSpeed / driveMotorReduction * wheelRadiusMeters;

  public static final double driveKp = 2.525;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.5;
  public static final double driveKv = 2 * PI * driveMotorReduction / driveGearbox.Kv;
  public static final double driveKa = 0.0;
  public static final double driveSimP = 0.3;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 10;
  public static final double turnMotorReduction = 26;
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  public static final boolean turnEncoderInverted = false;

  public static final double turnKp = 12.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 9.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = -PI;
  public static final double turnPIDMaxInput = PI;

  public static final double deadband = 0.1;

  public static final double headingKp = 5.0;
  public static final double headingKd = 0.4;
  public static final double headingToleranceRad = Units.degreesToRadians(2.0);

  public static final double ppTranslationKp = 14.0;
  public static final double ppTranslationKd = 0.0;
  public static final double ppRotationKp = 7.0;
  public static final double ppRotationKd = 0.0;

  public static final double dpadSpeedMetersPerSec = 0.5;

  public static final double ffStartDelay = 2.0;
  public static final double ffRampRate = 0.1;
  public static final double wheelRadiusMaxVelocity = 0.25;
  public static final double wheelRadiusRampRate = 0.05;

  public static final double robotMassKg = 51.00;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;

  public static final double rollThreshold = 10.0;
  public static final double pitchThreshold = 10.0;
}
