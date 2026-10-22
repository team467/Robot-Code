package first.robot.mechanisms.vision;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(Rotation2d.ZERO, Rotation2d.ZERO);
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance) {}

  public default void updateInputs(VisionIOInputs inputs) {}
}
