package first.robot.mechanisms.vision;

import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;

public final class VisionConstants {
  public static final Field aprilTagLayout = Field.loadField(Fields.FRC_2026_REBUILT_WELDED);

  public static final String camera0Name = "camera_0";
  public static final String camera1Name = "camera_1";
  public static final String camera2Name = "camera_2";
  public static final String camera3Name = "camera_3";

  private static final Rotation3d RotationCorrection =
      new Rotation3d(0, 0, Math.PI / 2);
  public static final Transform3d robotToCamera0 =
      new Transform3d(
          new Translation3d(
                  Units.inchesToMeters(-8.931),
                  Units.inchesToMeters(12.349),
                  Units.inchesToMeters(15.2025))
              .rotateBy(RotationCorrection),
          new Rotation3d(
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(201.229)));
  public static final Transform3d robotToCamera1 =
      new Transform3d(
          new Translation3d(
                  Units.inchesToMeters(8.931),
                  Units.inchesToMeters(12.349),
                  Units.inchesToMeters(15.2025))
              .rotateBy(RotationCorrection),
          new Rotation3d(
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(158.771)));

  public static final Transform3d robotToCamera2 =
      new Transform3d(
          new Translation3d(
                  Units.inchesToMeters(11.425),
                  Units.inchesToMeters(-2.361),
                  Units.inchesToMeters(20.5805))
              .rotateBy(RotationCorrection),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-30), Units.degreesToRadians(0)));

  public static final double maxAmbiguity = 0.3;
  public static final double maxZError = 0.75;

  public static final double linearStdDevBaseline = 0.15;
  public static final double angularStdDevBaseline = 0.25;

  public static final double[] cameraStdDevFactors =
      new double[] {
        1.0,
        1.0,
        1.0,
      };

  private VisionConstants() {}
}
