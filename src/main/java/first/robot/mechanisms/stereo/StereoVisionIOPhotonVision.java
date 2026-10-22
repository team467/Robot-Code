package first.robot.mechanisms.stereo;

import static first.robot.mechanisms.stereo.StereoVisionConstants.*;

import java.util.Comparator;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class StereoVisionIOPhotonVision implements StereoVisionIO {
  protected final PhotonCamera leftCamera;
  protected final PhotonCamera rightCamera;

  public StereoVisionIOPhotonVision(String leftName, String rightName) {
    leftCamera = new PhotonCamera(leftName);
    rightCamera = new PhotonCamera(rightName);
  }

  @Override
  public void updateInputs(StereoVisionIOInputs inputs) {
    inputs.leftConnected = leftCamera.isConnected();
    inputs.rightConnected = rightCamera.isConnected();

    List<PhotonPipelineResult> leftResults = leftCamera.getAllUnreadResults();
    if (!leftResults.isEmpty()) {
      PhotonPipelineResult latest = leftResults.get(leftResults.size() - 1);
      List<PhotonTrackedTarget> fuel = fuelTargets(latest);
      inputs.leftTimestamp = latest.getTimestampSeconds();
      inputs.leftYawDegrees = fuel.stream().mapToDouble(PhotonTrackedTarget::getYaw).toArray();
      inputs.leftPitchDegrees = fuel.stream().mapToDouble(PhotonTrackedTarget::getPitch).toArray();
      inputs.leftConfidence =
          fuel.stream().mapToDouble(PhotonTrackedTarget::getDetectedObjectConfidence).toArray();
    }

    List<PhotonPipelineResult> rightResults = rightCamera.getAllUnreadResults();
    if (!rightResults.isEmpty()) {
      PhotonPipelineResult latest = rightResults.get(rightResults.size() - 1);
      List<PhotonTrackedTarget> fuel = fuelTargets(latest);
      inputs.rightTimestamp = latest.getTimestampSeconds();
      inputs.rightYawDegrees = fuel.stream().mapToDouble(PhotonTrackedTarget::getYaw).toArray();
      inputs.rightPitchDegrees = fuel.stream().mapToDouble(PhotonTrackedTarget::getPitch).toArray();
      inputs.rightConfidence =
          fuel.stream().mapToDouble(PhotonTrackedTarget::getDetectedObjectConfidence).toArray();
    }
  }

  private static List<PhotonTrackedTarget> fuelTargets(PhotonPipelineResult result) {
    return result.getTargets().stream()
        .filter(t -> t.getDetectedObjectClassID() == fuelClassId)
        .filter(t -> t.getDetectedObjectConfidence() >= minConfidence)
        .sorted(Comparator.comparingDouble(PhotonTrackedTarget::getArea).reversed())
        .limit(maxFuelPerCamera)
        .toList();
  }
}
