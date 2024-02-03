package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.utils.TunableNumber;
import frc.robot.Constants;
import frc.robot.constants.controls.GearRatio;

public class DriveConstants {
  public static final double TRACK_WIDTH_X;
  public static final double TRACK_WIDTH_Y;
  public static final double MAX_LINEAR_SPEED;
  public static final double WHEEL_DIAMETER;
  public static final Rotation2d[] ABSOLUTE_ANGLE_OFFSET;
  public static final GearRatio DRIVE_GEAR_RATIO;
  public static final GearRatio TURN_GEAR_RATIO;
  public static final TunableNumber DRIVE_KS;
  public static final TunableNumber DRIVE_KV;
  public static final TunableNumber TURN_KP;
  public static final TunableNumber TURN_KD;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023 -> {
        TRACK_WIDTH_X = Units.inchesToMeters(12.75 * 2);
        TRACK_WIDTH_Y = Units.inchesToMeters(9.25 * 2);
        MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
        WHEEL_DIAMETER = Units.inchesToMeters(4);
        ABSOLUTE_ANGLE_OFFSET =
            new Rotation2d[] {
              Rotation2d.fromDegrees(-67.4),
              Rotation2d.fromDegrees(42.4),
              Rotation2d.fromDegrees(169.7),
              Rotation2d.fromDegrees(101.5),
            };
        DRIVE_GEAR_RATIO = new GearRatio(6.75, 1);
        TURN_GEAR_RATIO = new GearRatio(12.8, 1);
        DRIVE_KS = new TunableNumber("Drive/Module/DriveKS", 0.49385);
        DRIVE_KV = new TunableNumber("Drive/Module/DriveKV", 2.60818);
        TURN_KP = new TunableNumber("Drive/Module/TurnKP", 3.256);
        TURN_KD = new TunableNumber("Drive/Module/TurnKD", 0.05);
      }
      case ROBOT_2024A -> {
        TRACK_WIDTH_X = Units.inchesToMeters(11.5 * 2);
        TRACK_WIDTH_Y = Units.inchesToMeters(11.5 * 2);
        MAX_LINEAR_SPEED = Units.feetToMeters(16.0);
        WHEEL_DIAMETER = Units.inchesToMeters(4);
        ABSOLUTE_ANGLE_OFFSET =
            new Rotation2d[] {
              Rotation2d.fromDegrees(277.4),
              Rotation2d.fromDegrees(46.6),
              Rotation2d.fromDegrees(89.7),
              Rotation2d.fromDegrees(-137.7),
            };
        DRIVE_GEAR_RATIO = new GearRatio(6.12, 1);
        TURN_GEAR_RATIO = new GearRatio(12.8, 1);
        DRIVE_KS = new TunableNumber("Drive/Module/DriveKS", 0.49385);
        DRIVE_KV = new TunableNumber("Drive/Module/DriveKV", 2.60818);
        TURN_KP = new TunableNumber("Drive/Module/TurnKP", 4.5);
        TURN_KD = new TunableNumber("Drive/Module/TurnKD", 0.1);
      }
      case ROBOT_SIMBOT -> {
        TRACK_WIDTH_X = 0.65;
        TRACK_WIDTH_Y = 0.65;
        MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
        WHEEL_DIAMETER = Units.inchesToMeters(2);
        ABSOLUTE_ANGLE_OFFSET = new Rotation2d[] {new Rotation2d()};
        DRIVE_GEAR_RATIO = new GearRatio(6.75, 1);
        TURN_GEAR_RATIO = new GearRatio(12.8, 1);
        DRIVE_KS = new TunableNumber("Drive/Module/DriveKS", 0.15343);
        DRIVE_KV = new TunableNumber("Drive/Module/DriveKV", 5.27611);
        TURN_KP = new TunableNumber("Drive/Module/TurnKP", 23.0);
        TURN_KD = new TunableNumber("Drive/Module/TurnKD", 0.0);
      }
      default -> {
        TRACK_WIDTH_X = 0;
        TRACK_WIDTH_Y = 0;
        MAX_LINEAR_SPEED = 0;
        WHEEL_DIAMETER = 0;
        ABSOLUTE_ANGLE_OFFSET = new Rotation2d[] {};
        DRIVE_GEAR_RATIO = new GearRatio();
        TURN_GEAR_RATIO = new GearRatio();
        DRIVE_KS = new TunableNumber("Drive/Module/DriveKS");
        DRIVE_KV = new TunableNumber("Drive/Module/DriveKV");
        TURN_KP = new TunableNumber("Drive/Module/TurnKP");
        TURN_KD = new TunableNumber("Drive/Module/TurnKD");
      }
    }
  }
}
