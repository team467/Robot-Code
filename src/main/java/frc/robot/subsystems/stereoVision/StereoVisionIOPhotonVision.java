package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class StereoVisionIOPhotonVision implements StereoVisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  /**
  * Creates a PhotonVision camera input for object detection.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */

  public StereoVisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    camera.setPipelineIndex(StereoVisionConstants.yoloPipelineIndex);
  }

  @Override
  public Transform3d getRobotToCamera() {
    return robotToCamera;
  }

  @Override
  public void updateInputs(StereoVisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    for (var result : camera.getAllUnreadResults()) {
      inputs.timestampSeconds = result.getTimestampSeconds();
      List<PhotonTrackedTarget> targets = result.getTargets();
      inputs.targetClassIds = new int[targets.size()];
      inputs.targetYawDegrees = new double[targets.size()];
      inputs.targetPitchDegrees = new double[targets.size()];
      for (int i = 0; i < targets.size(); i++) {
        PhotonTrackedTarget target = targets.get(i);
        inputs.targetClassIds[i] = target.getDetectedObjectClassID();
        inputs.targetYawDegrees[i] = target.getYaw();
        inputs.targetPitchDegrees[i] = target.getPitch();
      }
    }
  }
}
