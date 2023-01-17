package frc.lib.io.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.io.vision.poseestimator.EstimatedRobotPose;
import frc.lib.io.vision.poseestimator.PhotonPoseEstimator;
import frc.lib.io.vision.poseestimator.PhotonPoseEstimator.PoseStrategy;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private boolean aprilTag = true;

  public VisionIOPhotonVision(String cameraName, Transform3d robotToCam) {
    camera = new PhotonCamera(cameraName);
    camera.setPipelineIndex(1);
    List<AprilTag> aprilTags = new ArrayList<>();
    aprilTags.add(
        new AprilTag(
            4,
            new Pose3d(
                new Translation3d(Units.inchesToMeters(125), 0, Units.inchesToMeters(25)),
                new Rotation3d())));
    poseEstimator =
        new PhotonPoseEstimator(
            new AprilTagFieldLayout(
                aprilTags, Units.inchesToMeters(130), Units.inchesToMeters(130)),
            PoseStrategy.AVERAGE_BEST_TARGETS,
            camera,
            robotToCam);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    var result = camera.getLatestResult();
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
}
