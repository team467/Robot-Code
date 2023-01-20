package frc.lib.io.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
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
      PhotonTrackedTarget target = result.getBestTarget();
      inputs.bestYaw = target.getYaw();
      inputs.bestPitch = target.getPitch();
      inputs.bestArea = target.getArea();
      inputs.bestSkew = target.getSkew();
      Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      inputs.bestBestCameraToTarget =
          new double[] {
            bestCameraToTarget.getTranslation().getX(),
            bestCameraToTarget.getTranslation().getZ(),
            bestCameraToTarget.getTranslation().getZ(),
            bestCameraToTarget.getRotation().getQuaternion().getW(),
            bestCameraToTarget.getRotation().getQuaternion().getX(),
            bestCameraToTarget.getRotation().getQuaternion().getY(),
            bestCameraToTarget.getRotation().getQuaternion().getZ(),
          };
      Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
      inputs.bestAltCameraToTarget =
          new double[] {
            alternateCameraToTarget.getTranslation().getX(),
            alternateCameraToTarget.getTranslation().getZ(),
            alternateCameraToTarget.getTranslation().getZ(),
            alternateCameraToTarget.getRotation().getQuaternion().getW(),
            alternateCameraToTarget.getRotation().getQuaternion().getX(),
            alternateCameraToTarget.getRotation().getQuaternion().getY(),
            alternateCameraToTarget.getRotation().getQuaternion().getZ(),
          };
      inputs.fiducialId = target.getFiducialId();
      inputs.bestPoseAmbiguity = target.getPoseAmbiguity();

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
