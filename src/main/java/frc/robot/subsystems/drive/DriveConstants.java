package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = Units.feetToMeters(16.6);
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(20.5);
  public static final double wheelBase = Units.inchesToMeters(20.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-0.8002675215350552);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.4471189677715302);
  public static final Rotation2d backLeftZeroRotation =
      new Rotation2d(3.027200698852539 - 2 * Math.PI);
  public static final Rotation2d backRightZeroRotation =
      new Rotation2d(3.0461928844451904 - 2 * Math.PI);

  // Device CAN IDs
  public static final int pigeonCanId = 17;

  public static final int frontLeftDriveCanId = 1;
  public static final int backLeftDriveCanId = 5;
  public static final int frontRightDriveCanId = 7;
  public static final int backRightDriveCanId = 3;

  public static final int frontLeftTurnCanId = 2;
  public static final int backLeftTurnCanId = 6;
  public static final int frontRightTurnCanId = 4;
  public static final int backRightTurnCanId = 8;

  public static final int frontLeftAbsoluteEncoderCanId = 21;
  public static final int backLeftAbsoluteEncoderCanId = 20;
  public static final int frontRightAbsoluteEncoderCanId = 18;
  public static final int backRightAbsoluteEncoderCanId = 19;

  // Drive motor configuration
  public static final SwerveModuleConstants.ClosedLoopOutputType driveClosedLoopOutput =
      SwerveModuleConstants.ClosedLoopOutputType.Voltage;
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2);
  public static final double driveMotorReduction = 6.12;
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0;
  public static final double driveKv = 0.1;
  public static final double driveKa = 0.0;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 12.8;
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = false;
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / turnMotorReduction; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = -Math.PI; // Radians
  public static final double turnPIDMaxInput = Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox,
              driveMotorReduction,
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
