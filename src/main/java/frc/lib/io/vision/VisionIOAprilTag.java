package frc.lib.io.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOAprilTag implements VisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  public VisionIOAprilTag(
      String cameraName, Transform3d robotToCam, AprilTagFieldLayout fieldLayout) {
    camera = new PhotonCamera(cameraName);
    poseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS, camera, robotToCam);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    PhotonPipelineResult result = camera.getLatestResult();
    inputs.hasTargets = result.hasTargets();
    if (result.hasTargets()) {
      inputs.targets = result.getTargets().stream().map(Optional::ofNullable).toList();
    } else {
      inputs.targets = List.of(Optional.empty());
    }
    inputs.latencyMillis = result.getLatencyMillis();
    inputs.timestampSeconds = result.getTimestampSeconds();
    inputs.estimatedPose = poseEstimator.update();
  }

  @Override
  public void setDriverMode(boolean driverMode) {
    camera.setDriverMode(driverMode);
  }

  @Override
  public void setPipelineIndex(int index) {
    camera.setPipelineIndex(index);
  }
}
