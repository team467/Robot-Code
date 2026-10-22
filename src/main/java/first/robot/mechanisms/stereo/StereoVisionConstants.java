package first.robot.mechanisms.stereo;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.util.Units;

public final class StereoVisionConstants {
  public static final String leftCameraName = "stereo_left";
  public static final String rightCameraName = "stereo_right";

  public static final double baselineMeters = 0.10;
  public static final double cameraForwardMeters = 0.30;
  public static final double cameraHeightMeters = 0.40;
  public static final double cameraPitchDownDegrees = 20.0;

  public static final Transform3d robotToLeftCamera =
      new Transform3d(
          cameraForwardMeters,
          baselineMeters / 2.0,
          cameraHeightMeters,
          new Rotation3d(0.0, Units.degreesToRadians(cameraPitchDownDegrees), 0.0));
  public static final Transform3d robotToRightCamera =
      new Transform3d(
          cameraForwardMeters,
          -baselineMeters / 2.0,
          cameraHeightMeters,
          new Rotation3d(0.0, Units.degreesToRadians(cameraPitchDownDegrees), 0.0));

  public static final int fuelClassId = 0;
  public static final double minConfidence = 0.3;
  public static final double fuelDiameterMeters = Units.inchesToMeters(5.91);

  public static final double maxRayMissMeters = 0.10;
  public static final double maxRangeMeters = 4.0;
  public static final double maxHeightErrorMeters = 0.15;
  public static final int maxFuelPerCamera = 10;

  public static final int simFuelCount = 40;
  public static final double simFieldMarginMeters = 0.5;
  public static final long simRandomSeed = 467;
  public static final int simResolutionWidth = 1280;
  public static final int simResolutionHeight = 800;
  public static final double simDiagonalFovDegrees = 90.0;
  public static final boolean simEnableCameraStreams = true;

  private StereoVisionConstants() {}
}
