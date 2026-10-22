package first.robot.mechanisms.stereo;

import java.util.Optional;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

public final class StereoTriangulation {
  public static record Ray(Translation3d origin, Translation3d direction) {}

  public static record Result(Translation3d point, double missDistanceMeters) {}

  private static final double PARALLEL_EPSILON = 1e-9;

  private StereoTriangulation() {}

  public static Ray rayInRobotFrame(
      Transform3d robotToCamera, double yawDegrees, double pitchDegrees) {
    Translation3d cameraDirection =
        new Translation3d(
            1.0,
            -Math.tan(Math.toRadians(yawDegrees)),
            Math.tan(Math.toRadians(pitchDegrees)));
    Translation3d direction = cameraDirection.rotateBy(robotToCamera.getRotation());
    return new Ray(robotToCamera.getTranslation(), direction.div(direction.getNorm()));
  }

  public static Optional<Result> triangulate(Ray a, Ray b) {
    Translation3d w0 = a.origin().minus(b.origin());
    double aa = a.direction().dot(a.direction());
    double ab = a.direction().dot(b.direction());
    double bb = b.direction().dot(b.direction());
    double aw = a.direction().dot(w0);
    double bw = b.direction().dot(w0);
    double denominator = aa * bb - ab * ab;
    if (Math.abs(denominator) < PARALLEL_EPSILON) {
      return Optional.empty();
    }
    double t = (ab * bw - bb * aw) / denominator;
    double s = (aa * bw - ab * aw) / denominator;
    if (t <= 0 || s <= 0) {
      return Optional.empty();
    }
    Translation3d onA = a.origin().plus(a.direction().times(t));
    Translation3d onB = b.origin().plus(b.direction().times(s));
    return Optional.of(new Result(onA.interpolate(onB, 0.5), onA.getDistance(onB)));
  }
}
