package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";
  public static String camera2Name = "camera_2"; //CHANGED

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(-11.528),
          Units.inchesToMeters(-7.289),
          Units.inchesToMeters(11.75),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(130)));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(-13.978),
          Units.inchesToMeters(0),
          Units.inchesToMeters(4.805),
          new Rotation3d(0.0, Units.degreesToRadians(-22), Math.PI));

   public static Transform3d robotToCamera2 =
      new Transform3d(
          Units.inchesToMeters(0), //CHANGE THIS LATER
          Units.inchesToMeters(0), //CHANGE THIS LATER
          Units.inchesToMeters(0), //CHANGE THIS LATER
          new Rotation3d(0.0, Units.degreesToRadians(-22), Math.PI)); //CHANGE THIS LATER        
 

  
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
        1.0    //CAMERA 2
      };
}
