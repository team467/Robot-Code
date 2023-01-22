package frc.lib.io.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
      List<PhotonTrackedTarget> targets = result.getTargets();
      for (int i = 0; i < 10; i++) {
        boolean targetExists = i + 1 > targets.size();
        inputs.yaws[i*2] = targetExists ? 0.0 : 1.0;
        inputs.yaws[i*2+1] = targetExists ? 0.0 : targets.get(i).getYaw();
        inputs.pitches[i*2] = targetExists ? 0.0 : 1.0;
        inputs.pitches[i*2+1] = targetExists ? 0.0 : targets.get(i).getPitch();
        inputs.areas[i*2] = targetExists ? 0.0 : 1.0;
        inputs.skews[i*2] = targetExists ? 0.0 : 1.0;
        inputs.fiducialIds[i*2] = targetExists ? 0L : 1L;
        inputs.bestCameraToTargets[i*7] = targetExists ? 0.0 : 1.0;
        inputs.bestCameraToTargets[i*7+1] = targetExists ? 0.0 : targets.get(i).getBestCameraToTarget().getX();
        inputs.bestCameraToTargets[i*7+2] = targetExists ? 0.0 : targets.get(i).getBestCameraToTarget().getY();
        inputs.bestCameraToTargets[i*7+3] = targetExists ? 0.0 : targets.get(i).getBestCameraToTarget().getZ();
        inputs.bestCameraToTargets[i*7+4] = targetExists ? 0.0 : targets.get(i).getBestCameraToTarget().getRotation().getQuaternion().getW();
        inputs.bestCameraToTargets[i*7+5] = targetExists ? 0.0 : targets.get(i).getBestCameraToTarget().getRotation().getQuaternion().getX();
        inputs.bestCameraToTargets[i*7+6] = targetExists ? 0.0 : targets.get(i).getBestCameraToTarget().getRotation().getQuaternion().getY();
        inputs.bestCameraToTargets[i*7+7] = targetExists ? 0.0 : targets.get(i).getBestCameraToTarget().getRotation().getQuaternion().getZ();
        inputs.altCameraToTargets[i*7] = targetExists ? 0.0 : 1.0;
        inputs.altCameraToTargets[i*7+1] = targetExists ? 0.0 : targets.get(i).getAlternateCameraToTarget().getX();
        inputs.altCameraToTargets[i*7+2] = targetExists ? 0.0 : targets.get(i).getAlternateCameraToTarget().getY();
        inputs.altCameraToTargets[i*7+3] = targetExists ? 0.0 : targets.get(i).getAlternateCameraToTarget().getZ();
        inputs.altCameraToTargets[i*7+4] = targetExists ? 0.0 : targets.get(i).getAlternateCameraToTarget().getRotation().getQuaternion().getW();
        inputs.altCameraToTargets[i*7+5] = targetExists ? 0.0 : targets.get(i).getAlternateCameraToTarget().getRotation().getQuaternion().getX();
        inputs.altCameraToTargets[i*7+6] = targetExists ? 0.0 : targets.get(i).getAlternateCameraToTarget().getRotation().getQuaternion().getY();
        inputs.altCameraToTargets[i*7+7] = targetExists ? 0.0 : targets.get(i).getAlternateCameraToTarget().getRotation().getQuaternion().getZ();
      }

      Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update();
      if (estimatedRobotPose.isPresent()) {
        inputs.estimatedPose =
            new double[] {
              estimatedRobotPose.get().estimatedPose.getTranslation().getX(),
              estimatedRobotPose.get().estimatedPose.getTranslation().getZ(),
              estimatedRobotPose.get().estimatedPose.getTranslation().getZ(),
              estimatedRobotPose.get().estimatedPose.getRotation().getQuaternion().getW(),
              estimatedRobotPose.get().estimatedPose.getRotation().getQuaternion().getX(),
              estimatedRobotPose.get().estimatedPose.getRotation().getQuaternion().getY(),
              estimatedRobotPose.get().estimatedPose.getRotation().getQuaternion().getZ(),
            };
        inputs.estimatedPoseTimestamp = estimatedRobotPose.get().timestampSeconds;
      } else {
        inputs.estimatedPose = new double[] {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
        inputs.estimatedPoseTimestamp = -1;
      }
    }
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
