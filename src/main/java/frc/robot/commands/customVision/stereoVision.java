package frc.robot.commands.customVision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class stereoVision {
  /** Maximum time (seconds) between the two cameras' detections before they're considered stale. */
  private static final double MAX_TIME_DELTA_SECONDS = 0.10;

  private static final double MAX_RAY_PARALLELISM = 0.999;

  private final PhotonCamera cameraA;
  private final PhotonCamera cameraB;
  private final Transform3d robotToCameraA;
  private final Transform3d robotToCameraB;

  private PhotonTrackedTarget latestTargetA;
  private double latestTimestampA = -1.0;
  private PhotonTrackedTarget latestTargetB;
  private double latestTimestampB = -1.0;

  /**
   * Creates a new StereoVision pair.
   *
   * @param nameA The configured PhotonVision name of the first camera.
   * @param robotToCameraA The 3D position of the first camera relative to the robot.
   * @param nameB The configured PhotonVision name of the second camera.
   * @param robotToCameraB The 3D position of the second camera relative to the robot.
   */
  public stereoVision(
      String nameA, Transform3d robotToCameraA, String nameB, Transform3d robotToCameraB) {
    this.cameraA = new PhotonCamera(nameA);
    this.robotToCameraA = robotToCameraA;
    this.cameraB = new PhotonCamera(nameB);
    this.robotToCameraB = robotToCameraB;
  }

  /** Returns whether both cameras are currently connected. */
  public boolean isConnected() {
    return cameraA.isConnected() && cameraB.isConnected();
  }

  public void updateInputs(int classId) {
    for (PhotonPipelineResult result : cameraA.getAllUnreadResults()) {
      PhotonTrackedTarget best = bestTarget(result.getTargets(), classId);
      if (best != null) {
        latestTargetA = best;
        latestTimestampA = result.getTimestampSeconds();
      }
    }

    for (PhotonPipelineResult result : cameraB.getAllUnreadResults()) {
      PhotonTrackedTarget best = bestTarget(result.getTargets(), classId);
      if (best != null) {
        latestTargetB = best;
        latestTimestampB = result.getTimestampSeconds();
      }
    }
  }

  /**
   * Returns the triangulated position of the tracked object relative to the robot's origin, if both
   * cameras have a sufficiently recent, non-degenerate detection to triangulate from.
   */
  public Optional<StereoObservation> getObjectPosition() {
    if (latestTargetA == null || latestTargetB == null) {
      return Optional.empty();
    }
    if (Math.abs(latestTimestampA - latestTimestampB) > MAX_TIME_DELTA_SECONDS) {
      return Optional.empty();
    }

    Translation3d originA = robotToCameraA.getTranslation();
    Translation3d originB = robotToCameraB.getTranslation();
    Translation3d directionA = bearingRay(latestTargetA, robotToCameraA);
    Translation3d directionB = bearingRay(latestTargetB, robotToCameraB);

    if (Math.abs(directionA.dot(directionB)) > MAX_RAY_PARALLELISM) {
      return Optional.empty();
    }

    Translation3d[] closestPoints =
        closestPointsBetweenRays(originA, directionA, originB, directionB);
    Translation3d estimatedPosition = closestPoints[0].interpolate(closestPoints[1], 0.5);
    double triangulationError = closestPoints[0].getDistance(closestPoints[1]);

    return Optional.of(
        new StereoObservation(
            (latestTimestampA + latestTimestampB) / 2.0,
            estimatedPosition,
            triangulationError,
            latestTargetA.getDetectedObjectClassID()));
  }

  /** Returns the highest-area target matching classId (or any target if classId is negative). */
  private static PhotonTrackedTarget bestTarget(List<PhotonTrackedTarget> targets, int classId) {
    PhotonTrackedTarget best = null;
    for (PhotonTrackedTarget target : targets) {
      if (classId >= 0 && target.getDetectedObjectClassID() != classId) {
        continue;
      }
      if (best == null || target.getArea() > best.getArea()) {
        best = target;
      }
    }
    return best;
  }

  private static Translation3d bearingRay(PhotonTrackedTarget target, Transform3d robotToCamera) {
    Rotation3d bearingInCameraFrame =
        new Rotation3d(
            0.0,
            -Units.degreesToRadians(target.getPitch()),
            Units.degreesToRadians(target.getYaw()));
    Translation3d rayInCameraFrame =
        new Translation3d(1.0, 0.0, 0.0).rotateBy(bearingInCameraFrame);
    return rayInCameraFrame.rotateBy(robotToCamera.getRotation());
  }

  private static Translation3d[] closestPointsBetweenRays(
      Translation3d originA,
      Translation3d directionA,
      Translation3d originB,
      Translation3d directionB) {
    Translation3d originDelta = originA.minus(originB);
    double b = directionA.dot(directionB);
    double d = directionA.dot(originDelta);
    double e = directionB.dot(originDelta);
    double denominator = 1.0 - b * b;

    double tA = (b * e - d) / denominator;
    double tB = (e - b * d) / denominator;

    Translation3d closestA = originA.plus(directionA.times(tA));
    Translation3d closestB = originB.plus(directionB.times(tB));
    return new Translation3d[] {closestA, closestB};
  }

  /**
   * A single triangulated object detection.
   *
   * @param timestamp Timestamp of the observation, averaged across both cameras (seconds).
   * @param translation Estimated position of the object relative to the robot's origin.
   * @param triangulationError Distance between the two rays' closest points; a rough measure of
   *     triangulation quality (larger = less trustworthy).
   * @param classId The object-detection class ID that was tracked.
   */
  public record StereoObservation(
      double timestamp, Translation3d translation, double triangulationError, int classId) {}
}
