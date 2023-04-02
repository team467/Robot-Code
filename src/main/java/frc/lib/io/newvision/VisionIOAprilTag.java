package frc.lib.io.newvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOAprilTag implements VisionIO {
  private final PhotonPoseEstimator poseEstimator;
  private final RawSubscriber rawBytesEntry;

  public VisionIOAprilTag(
      String cameraName, Transform3d robotToCam, AprilTagFieldLayout fieldLayout) {
    rawBytesEntry =
        NetworkTableInstance.getDefault()
            .getTable("photonvision")
            .getSubTable(cameraName)
            .getRawTopic("rawBytes")
            .subscribe(
                "rawBytes", new byte[] {}, PubSubOption.periodic(0.01), PubSubOption.sendAll(true));
    poseEstimator =
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, null, robotToCam);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.rawEntry = rawBytesEntry.get(new byte[] {});
    Packet packet = new Packet(inputs.rawEntry);
    PhotonPipelineResult result = new PhotonPipelineResult();
    if (packet.getSize() >= 1) result.createFromPacket(packet);
    result.setTimestampSeconds(
        (rawBytesEntry.getLastChange() / 1e6) - result.getLatencyMillis() / 1e3);
    inputs.estimatedPose = poseEstimator.update(result);
  }
}
