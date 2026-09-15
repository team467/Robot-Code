package frc.robot.commands.customVision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class ballVision extends SubsystemBase {

  // TODO: set to the actual game piece diameter for this year's ball
  private static final double BALL_DIAMETER_METERS = Units.inchesToMeters(5.91);

  // Set to the video mode width your camera is actually streaming at
  private static final int IMAGE_WIDTH_PIXELS = SwyftUsbCameraConstants.NATIVE_WIDTH_PIXELS;

  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final List<Pose3d> ballPoses = new ArrayList<>();

  public ballVision(String name) {
    this(name, new Transform3d());
  }

  public ballVision(String name, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.recordOutput("BallVision/Connected", camera.isConnected());
    Logger.recordOutput("BallVision/BallPoses", ballPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("BallVision/HasTarget", hasTarget());
    Logger.recordOutput("BallVision/BallCount", getBallCount());
    getClosestBallPose().ifPresent(pose -> Logger.recordOutput("BallVision/ClosestBallPose", pose));
  }

  public void updateInputs() {
    ballPoses.clear();

    if (!camera.isConnected()) {
      return;
    }

    var newestObservations = camera.getLatestResult().getTargets();

    // Prefer PhotonVision's live calibration if available, otherwise fall back
    // to the factory value scaled to our resolution (same trick as getFocalLengthPixels above).
    double focalLengthPixels =
        camera
            .getCameraMatrix()
            .map(m -> m.get(0, 0))
            .orElse(
                SwyftUsbCameraConstants.FACTORY_FX_AT_NATIVE_RESOLUTION
                    * (IMAGE_WIDTH_PIXELS / (double) SwyftUsbCameraConstants.NATIVE_WIDTH_PIXELS));

    for (PhotonTrackedTarget target : newestObservations) {
      // --- 1. Apparent size -> distance ---
      // Bounding box of the min-area rect gives us the ball's on-screen diameter in pixels.
      List<TargetCorner> corners = target.getMinAreaRectCorners();
      double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
      double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
      for (TargetCorner corner : corners) {
        minX = Math.min(minX, corner.x);
        maxX = Math.max(maxX, corner.x);
        minY = Math.min(minY, corner.y);
        maxY = Math.max(maxY, corner.y);
      }
      double pixelWidth = maxX - minX;
      double pixelHeight = maxY - minY;
      // Average width/height so partial occlusion or a slightly non-circular contour
      // doesn't throw the estimate off as badly as picking just one axis.
      double apparentDiameterPixels = (pixelWidth + pixelHeight) / 2.0;

      if (apparentDiameterPixels <= 0) {
        continue; // degenerate detection, skip it
      }

      // Similar-triangles: real diameter / apparent diameter = distance / focal length
      double distanceMeters = (BALL_DIAMETER_METERS * focalLengthPixels) / apparentDiameterPixels;

      // --- 2. Screen location -> direction ---
      // PhotonVision already converts pixel position into yaw/pitch angles using the
      // camera's calibration, correcting for lens distortion.
      double yawRadians = Units.degreesToRadians(target.getYaw());
      double pitchRadians = Units.degreesToRadians(target.getPitch());

      // Spherical -> Cartesian, in the camera's coordinate frame
      // (x forward, y left, z up, matching WPILib convention).
      double x = distanceMeters * Math.cos(yawRadians) * Math.cos(pitchRadians);
      double y = distanceMeters * Math.sin(yawRadians) * Math.cos(pitchRadians);
      double z = distanceMeters * Math.sin(pitchRadians);

      Transform3d cameraToBall = new Transform3d(new Translation3d(x, y, z), new Rotation3d());

      // --- 3. Camera space -> robot space ---
      Pose3d ballPoseRelativeToRobot =
          new Pose3d().transformBy(robotToCamera).transformBy(cameraToBall);

      ballPoses.add(ballPoseRelativeToRobot);
    }
  }

  public List<Pose3d> getBallPoses() {
    return new ArrayList<>(ballPoses);
  }

  public boolean hasTarget() {
    return !ballPoses.isEmpty();
  }

  public int getBallCount() {
    return ballPoses.size();
  }

  public Optional<Pose3d> getClosestBallPose() {
    return ballPoses.stream().min(Comparator.comparingDouble(p -> p.getTranslation().getNorm()));
  }

  public Optional<Translation2d> getClosestBallTranslation2d() {
    return getClosestBallPose().map(p -> p.getTranslation().toTranslation2d());
  }

  public Optional<Double> getDistanceToClosestBall() {
    return getClosestBallTranslation2d().map(Translation2d::getNorm);
  }

  public Optional<Rotation2d> getAngleToClosestBall() {
    return getClosestBallTranslation2d().map(t -> new Rotation2d(t.getX(), t.getY()));
  }
}
