package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class StereoVisionConstants {
    public static final String camera0Name = "stereo_camera_0";
    public static final String camera1Name = "stereo_camera_1";

    // Configure this PhotonVision pipeline to use the built-in Fuel v11n model.
    public static final int yoloPipelineIndex = 0;
    // Fuel v11n has one class, so its class ID is 0.
    public static final int fuelObjectClassId = 0;

    private static final Rotation3d rotationCorrection =
            new Rotation3d(0, 0, Math.PI / 2);

    public static final double maxObservationAgeSeconds = 0.25;
    public static final double maxTimestampDifferenceSeconds = 0.10;
    public static final double minimumTriangulationDenominator = 1e-4;
    public static final double maximumRaySeparationMeters = 0.25;

    //replace these sample poses with measured stereo camera mounting poses.
    public static final Transform3d robotToCamera0 =
      new Transform3d( // front camera
          new Translation3d(
              Units.inchesToMeters(8.779),
              Units.inchesToMeters(10.445),
              Units.inchesToMeters(27.152 + 1.75))
              .rotateBy(rotationCorrection),
          new Rotation3d(
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(-25.2),
              Units.degreesToRadians(0.0)));

    public static final Transform3d robotToCamera1 =
      new Transform3d( // rear left camera
          new Translation3d(
              Units.inchesToMeters(9.562), // x
              Units.inchesToMeters(10.974), // y
              Units.inchesToMeters(17.035 + 1.75)) // z
              .rotateBy(rotationCorrection),
          new Rotation3d(
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(-11.32),
              Units.degreesToRadians(155.3)));
}
