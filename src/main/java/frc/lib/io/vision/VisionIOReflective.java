package frc.lib.io.vision;

import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

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
      inputs.targets =
          result.getTargets().stream().map(Optional::ofNullable).toList();
    } else {
      inputs.targets = List.of(Optional.empty());
    }
    inputs.latencyMillis = result.getLatencyMillis();
    inputs.timestampSeconds = result.getTimestampSeconds();
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
