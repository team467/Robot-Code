package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class StereoVisionConstants {
  // TODO: chnange names accordingly
  public static String camera0Name = "stereo_camera_0";
  public static String camera1Name = "stereo_camera_1";

  private static Rotation3d RotationCorrection =
      new Rotation3d(0, 0, Math.PI / 2); // 90 degree roation around z-axis

  // TODO: get correct coordinates for cams
  public static Transform3d robotToCamera0 =
      new Transform3d( // left stereo camera
          new Translation3d(
                  Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0))
              .rotateBy(RotationCorrection),
          new Rotation3d(
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(0.0)));

  public static Transform3d robotToCamera1 =
      new Transform3d( // right stereo camera
          new Translation3d(
                  Units.inchesToMeters(0.0), // x
                  Units.inchesToMeters(0.0), // y
                  Units.inchesToMeters(0.0)) // z
              .rotateBy(RotationCorrection),
          new Rotation3d(
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(0.0)));

  // Basic filtering thresholds
  public static double minDetectionConfidence = 0.3; // TODO: arbitrary - chnage based on testing
}
