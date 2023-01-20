package frc.lib.io.vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOReflective implements VisionIO {
  private final PhotonCamera camera;

  public VisionIOReflective(String cameraName) {
    camera = new PhotonCamera(cameraName);
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
