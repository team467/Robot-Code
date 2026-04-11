package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout();

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";
  public static String camera2Name = "camera_2";
  public static String camera3Name = "camera_3";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)

  private static Rotation3d RotationCorrection =
      new Rotation3d(0, 0, Math.PI / 2); // 90 degree roation around z-axis
  public static Transform3d robotToCamera0 =
      new Transform3d( // front camera
          new Translation3d(
                  Units.inchesToMeters(-8.931),
                  Units.inchesToMeters(12.349),
                  Units.inchesToMeters(15.2025))
              .rotateBy(RotationCorrection),
          new Rotation3d(
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(-158.771)));
  public static Transform3d robotToCamera1 =
      new Transform3d( // rear left camera
          new Translation3d(
                  Units.inchesToMeters(8.931), // x
                  Units.inchesToMeters(12.349), // y
                  Units.inchesToMeters(15.2025)) // z
              .rotateBy(RotationCorrection),
          new Rotation3d(
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(-201.229)));

  public static Transform3d robotToCamera2 =
      new Transform3d( // rear right camera
          new Translation3d(
                  Units.inchesToMeters(11.425),
                  Units.inchesToMeters(-2.361),
                  Units.inchesToMeters(20.5805))
              .rotateBy(RotationCorrection),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-30), Units.degreesToRadians(0)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;
  // PhotonEstimator strategy

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.15; // Meters
  public static double angularStdDevBaseline = 0.25; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, // CAMERA 2
      };
}
