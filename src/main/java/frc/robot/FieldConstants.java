package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;

/**
 * Contains various field dimensions and useful reference points for REBUILT 2026. Dimensions are in
 * meters, and sets of corners start in the lower left moving clockwise.
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall. Use the {@link frc.lib.utils.AllianceFlipUtil#apply(Translation2d)} and {@link
 * frc.lib.utils.AllianceFlipUtil#apply(Pose2d)} methods to flip these values based on the current
 * alliance color.
 */
public class FieldConstants {
  // Overall field dimensions
  public static final double fieldLength = Units.inchesToMeters(651.2);
  public static final double fieldWidth = Units.inchesToMeters(317.7);

  // Zone dimensions
  public static final double allianceZoneDepth = Units.inchesToMeters(158.6);
  public static final double neutralZoneDepth = Units.inchesToMeters(283.0);
  public static final double robotStartingLineOffset = Units.inchesToMeters(24.0);

  // Fuel specifications
  public static final double fuelDiameter = Units.inchesToMeters(5.91);

  public static class Hub {
    // Hub is centered between two bumps, 158.6" from alliance wall
    public static final Translation2d blueCenter =
        new Translation2d(Units.inchesToMeters(158.6), fieldWidth / 2.0);
    public static final Translation2d redCenter =
        new Translation2d(fieldLength - Units.inchesToMeters(158.6), fieldWidth / 2.0);

    public static final double size = Units.inchesToMeters(47.0); // Square hub
    public static final double openingSize = Units.inchesToMeters(41.7); // Hexagonal opening
    public static final double openingHeight = Units.inchesToMeters(72.0); // Front edge height

    // Hub center face poses (facing toward the hub opening)
    public static final Pose2d blueCenterFace = new Pose2d(blueCenter, Rotation2d.fromDegrees(0));
    public static final Pose2d redCenterFace = new Pose2d(redCenter, Rotation2d.fromDegrees(180));
  }

  public static class Bumps {
    public static final double width = Units.inchesToMeters(73.0);
    public static final double depth = Units.inchesToMeters(44.4);
    public static final double height = Units.inchesToMeters(6.513);

    // Bumps are on either side of the hub
    public static final double offsetFromHubCenter = Units.inchesToMeters(47.0 / 2.0 + 73.0 / 2.0);
  }

  public static class Trenches {
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double clearanceWidth = Units.inchesToMeters(50.34);
    public static final double clearanceHeight = Units.inchesToMeters(22.25);

    // There are 4 trenches total - positions would need to be determined from field drawings
  }

  public static class Tower {
    public static final double width = Units.inchesToMeters(49.25);
    public static final double depth = Units.inchesToMeters(45.0);
    public static final double totalHeight = Units.inchesToMeters(78.25);

    // Tower base
    public static final double baseWidth = Units.inchesToMeters(39.0);
    public static final double baseDepth = Units.inchesToMeters(45.18);

    // Uprights
    public static final double uprightHeight = Units.inchesToMeters(72.1);
    public static final double uprightSpacing = Units.inchesToMeters(32.25);

    // Rungs (center heights from floor)
    public static final double lowRungHeight = Units.inchesToMeters(27.0);
    public static final double midRungHeight = Units.inchesToMeters(45.0);
    public static final double highRungHeight = Units.inchesToMeters(63.0);
    public static final double rungSpacing = Units.inchesToMeters(18.0); // Center to center
    public static final double rungDiameter = Units.inchesToMeters(1.66); // OD of pipe
    public static final double rungExtension =
        Units.inchesToMeters(5.875); // Extension from upright

    // Tower center positions (one per alliance, centered on alliance wall)
    public static final Translation2d blueCenter =
        new Translation2d(Units.inchesToMeters(0), fieldWidth / 2.0);
    public static final Translation2d redCenter = new Translation2d(fieldLength, fieldWidth / 2.0);

    // Tower face poses (facing toward the field)
    public static final Pose2d blueCenterFace = new Pose2d(blueCenter, Rotation2d.fromDegrees(0));
    public static final Pose2d redCenterFace = new Pose2d(redCenter, Rotation2d.fromDegrees(180));
  }

  public static class Depot {
    public static final double width = Units.inchesToMeters(42.0);
    public static final double depth = Units.inchesToMeters(27.0);
    public static final double barrierWidth = Units.inchesToMeters(3.0);
    public static final double barrierHeight = Units.inchesToMeters(1.125); // With hook fastener

    // One depot per alliance along the alliance wall
    // Exact positions would need to be determined from field drawings
  }

  public static class Outpost {
    public static final double width = Units.inchesToMeters(31.8);
    public static final double areaWidth = Units.inchesToMeters(71.0);
    public static final double areaDepth = Units.inchesToMeters(134.0);

    // Upper opening (where fuel passes to field)
    public static final double upperOpeningWidth = Units.inchesToMeters(31.8);
    public static final double upperOpeningHeight = Units.inchesToMeters(7.0);
    public static final double upperOpeningBottomHeight = Units.inchesToMeters(28.1);

    // Lower opening (where robots push fuel into corral)
    public static final double lowerOpeningWidth = Units.inchesToMeters(32.0);
    public static final double lowerOpeningHeight = Units.inchesToMeters(7.0);
    public static final double lowerOpeningBottomHeight = Units.inchesToMeters(1.88);

    // Corral dimensions
    public static final double corralWidth = Units.inchesToMeters(35.8);
    public static final double corralDepth = Units.inchesToMeters(37.6);
    public static final double corralHeight = Units.inchesToMeters(8.13);
    public static final double corralTapeDistance = Units.inchesToMeters(12.7);
    public static final double chuteTapeDistance = Units.inchesToMeters(12.9);
  }

  public static class AprilTags {
    public static final double tagSize = Units.inchesToMeters(8.125);
    public static final double panelSize = Units.inchesToMeters(10.5);

    // AprilTag heights (center of tag from floor)
    public static final double hubTagHeight = Units.inchesToMeters(44.25);
    public static final double towerWallTagHeight = Units.inchesToMeters(21.75);
    public static final double outpostTagHeight = Units.inchesToMeters(21.75);
    public static final double trenchTagHeight = Units.inchesToMeters(35.0);

    // Tag IDs
    public static class TagIDs {
      // Hub tags: IDs vary by face (see field drawings for specific layout)
      // Tower wall tags
      public static final int[] blueTowerWall = {15, 16};
      public static final int[] redTowerWall = {31, 32};

      // Outpost tags
      public static final int[] blueOutpost = {13, 14};
      public static final int[] redOutpost = {29, 30};

      // Trench tags (4 trenches total)
      // Specific IDs would need to be determined from field drawings
    }

    // Tag positions would be populated here based on actual field layout
    // This would typically be done in a static initializer block
    public static final List<Pose3d> tagPoses = new ArrayList<>();

    static {
      // Initialize tag poses based on field drawings
      // Example structure (actual positions TBD from official drawings):
      /*
      tagPoses.add(new Pose3d(
          new Translation3d(x, y, hubTagHeight),
          new Rotation3d(0, 0, rotation)));
      */
    }
  }

  public static class DriverStation {
    public static final double baseHeight = Units.inchesToMeters(36.8);
    public static final double shelfWidth = Units.inchesToMeters(69.0);
    public static final double shelfDepth = Units.inchesToMeters(12.25);
    public static final double hookAndLoopLength = Units.inchesToMeters(54.0);
    public static final double sponsorPanelHeight = Units.inchesToMeters(6.0);
  }

  public static class StagingPositions {
    // Starting positions along the alliance wall
    // These would typically be where robots are placed before match start
    // Exact positions would need to be determined from field drawings and team strategy

    public static final Pose2d blueLeft =
        new Pose2d(
            Units.inchesToMeters(robotStartingLineOffset),
            Units.inchesToMeters(fieldWidth * 0.75),
            Rotation2d.fromDegrees(0));

    public static final Pose2d blueCenter =
        new Pose2d(
            Units.inchesToMeters(robotStartingLineOffset),
            fieldWidth / 2.0,
            Rotation2d.fromDegrees(0));

    public static final Pose2d blueRight =
        new Pose2d(
            Units.inchesToMeters(robotStartingLineOffset),
            Units.inchesToMeters(fieldWidth * 0.25),
            Rotation2d.fromDegrees(0));
  }

  public static class RobotDimensions {
    // Maximum robot dimensions per rules
    public static final double maxHeight = Units.inchesToMeters(30.0);

    // Bumper specifications
    public static final double bumperMinCoverage = Units.inchesToMeters(5.0); // Per corner
    public static final double bumperMaxExtension = Units.inchesToMeters(4.0);
    public static final double bumperMinPadding = Units.inchesToMeters(2.25);
    public static final double bumperMinHeight = Units.inchesToMeters(4.5);
    public static final double bumperMinHeightFromFloor = Units.inchesToMeters(2.5);
    public static final double bumperMaxHeightFromFloor = Units.inchesToMeters(5.75);
  }

  /** Climbing levels for the tower */
  public enum ClimbLevel {
    NONE(0.0),
    LOW(Tower.lowRungHeight),
    MID(Tower.midRungHeight),
    HIGH(Tower.highRungHeight);

    ClimbLevel(double height) {
      this.height = height;
    }

    public final double height;
  }
}
