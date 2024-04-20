package frc.lib.io.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.FieldConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * PhotonVision-based implementation of the VisionIO interface.
 *
 * <p>Adapted from
 * https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/swervedriveposeestsim/src/main/java/frc/robot/Vision.java
 */
public class VisionIOPhotonVision implements VisionIO {
  private static final int EXPIRATION_COUNT = 5;

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private final boolean[] tagsSeen;
  private double lastTimestamp = 0;
  private int cyclesWithNoResults = 0;

  /**
   * Creates a new VisionIOPhotonVision object.
   *
   * @param cameraName the name of the PhotonVision camera to use; the name must be unique
   */
  public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(cameraName);
    this.photonEstimator =
        new PhotonPoseEstimator(
            FieldConstants.aprilTags,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            robotToCamera);

    // the index of the array corresponds to the tag ID; so, add one since there is no tag ID 0
    this.tagsSeen = new boolean[FieldConstants.aprilTags.getTags().size() + 1];
  }

  /**
   * Updates the specified VisionIOInputs object with the latest data from the camera.
   *
   * @param inputs the VisionIOInputs object to update with the latest data from the camera
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Optional<EstimatedRobotPose> visionEstimate = this.photonEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();

    this.cyclesWithNoResults++;

    boolean newResult = Math.abs(latestTimestamp - lastTimestamp) > 1e-5;
    if (newResult) {
      double minAmbiguity = 10;
      for (PhotonTrackedTarget target : camera.getLatestResult().getTargets()) {
        if (target.getPoseAmbiguity() < minAmbiguity) {
          minAmbiguity = target.getPoseAmbiguity();
        }
      }
      inputs.minAmbiguity = minAmbiguity;

      visionEstimate.ifPresent(
          estimate -> {
            inputs.estimatedRobotPose = estimate.estimatedPose;
            inputs.estimatedRobotPoseTimestamp = estimate.timestampSeconds;
            for (int i = 0; i < this.tagsSeen.length; i++) {
              this.tagsSeen[i] = false;
            }
            for (int i = 0; i < estimate.targetsUsed.size(); i++) {
              this.tagsSeen[estimate.targetsUsed.get(i).getFiducialId()] = true;
            }
            inputs.tagsSeen = this.tagsSeen;
            inputs.lastCameraTimestamp = latestTimestamp;
            lastTimestamp = latestTimestamp;
            this.cyclesWithNoResults = 0;
          });
    }

    // if no tags have been seen for the specified number of cycles, clear the array
    if (this.cyclesWithNoResults == EXPIRATION_COUNT) {
      for (int i = 0; i < this.tagsSeen.length; i++) {
        this.tagsSeen[i] = false;
      }
      inputs.tagsSeen = this.tagsSeen;
    }
  }
}
