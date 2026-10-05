package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class StereoVisionConstants {
  // TODO: chnange names accordingly
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  //TODO: get correct
  public static Transform3d robotToCamera0 =
      new Transform3d( // front camera
          new Translation3d(
              Units.inchesToMeters(8.779),
              Units.inchesToMeters(10.445),
              Units.inchesToMeters(27.152 + 1.75))
              .rotateBy(RotationCorrection),
          new Rotation3d(
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(-25.2),
              Units.degreesToRadians(0.0)));

  public static Transform3d robotToCamera1 =
      new Transform3d( // rear left camera
          new Translation3d(
              Units.inchesToMeters(9.562), // x
              Units.inchesToMeters(10.974), // y
              Units.inchesToMeters(17.035 + 1.75)) // z
              .rotateBy(RotationCorrection),
          new Rotation3d(
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(-11.32),
              Units.degreesToRadians(155.3)));
}
