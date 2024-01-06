package frc.lib.io.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/**
 * The hardware abstraction interface for a PhotonVision-based co-processor that provides
 * PhotonPipelineResult objects. There is a one-to-one relationship between each VisionIO object and
 * each co-processor (e.g., Raspberry Pi) running PhotonVision.
 *
 * <p>In the future, this interface may be further abstracted to not be coupled to PhotonVision.
 * Currently, the abstraction is used to simulate vision.
 */
public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    Pose3d estimatedRobotPose = new Pose3d();
    double estimatedRobotPoseTimestamp = 0.0;
    int[] estimatedRobotPoseTags = new int[] {};
    double lastCameraTimestamp = 0.0;
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs the inputs to update
   */
  default void updateInputs(VisionIOInputs inputs) {}
}
