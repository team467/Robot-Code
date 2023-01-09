package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Map;

/**
 * Contains various field dimensions and useful reference points.
 *
 * <p>All translations and poses are stored with the origin at the bottom point on the BLUE ALLIANCE
 * wall. Use the {@link #allianceFlip(Translation2d)} and {@link #allianceFlip(Pose2d)} methods to
 * flip these values based on the current alliance color.
 */
public final class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(651.25);
  public static final double fieldWidth = Units.inchesToMeters(315.5);
  public static final double tapeWidth = Units.inchesToMeters(2.0);

  /** Dimensions for community and charging station, including the tape. */
  public static final class Community { // TODO
  }

  /** Dimensions for grids and nodes */
  public static final class Grids { // TODO
  }

  // Dimensions for loading zone and substations, including the tape
  public static final class LoadingZone {}

  /** Locations of staged game pieces */
  public static final class StagingLocations { // TODO
  }

  // TODO: Replace when
  // https://github.com/wpilibsuite/allwpilib/commit/babb0c1fcf060df44713c96d552fe33c6f337dd8 is
  // added to stable WPILib
  /** AprilTag locations (do not flip for red alliance) */
  public static final Map<Integer, Pose3d> aprilTags =
      Map.of(
          1,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(42.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI)),
          2,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(108.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI)),
          3,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI)),
          4,
          new Pose3d(
              Units.inchesToMeters(636.96),
              Units.inchesToMeters(265.74),
              Units.inchesToMeters(27.38),
              new Rotation3d(0.0, 0.0, Math.PI)),
          5,
          new Pose3d(
              Units.inchesToMeters(14.25),
              Units.inchesToMeters(265.74),
              Units.inchesToMeters(27.38),
              new Rotation3d()),
          6,
          new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
              Units.inchesToMeters(18.22),
              new Rotation3d()),
          7,
          new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(108.19),
              Units.inchesToMeters(18.22),
              new Rotation3d()),
          8,
          new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(42.19),
              Units.inchesToMeters(18.22),
              new Rotation3d()));

  /**
   * Flips a translation to the correct side of the field based on the current alliance color. By
   * default, all translations and poses in {@link FieldConstants} are stored with the origin at the
   * bottom point on the BLUE ALLIANCE wall.
   */
  public static Translation2d allianceFlip(Translation2d translation) {
    if (DriverStation.getAlliance() == Alliance.Red) {
      return new Translation2d(fieldLength - translation.getX(), translation.getY());
    } else {
      return translation;
    }
  }

  /**
   * Flips a pose to the correct side of the field based on the current alliance color. By default,
   * all translations and poses in {@link FieldConstants} are stored with the origin at the bottom
   * point on the BLUE ALLIANCE wall.
   */
  public static Pose2d allianceFlip(Pose2d pose) {
    if (DriverStation.getAlliance() == Alliance.Red) {
      return new Pose2d(
          fieldLength - pose.getX(),
          pose.getY(),
          new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    } else {
      return pose;
    }
  }
}
