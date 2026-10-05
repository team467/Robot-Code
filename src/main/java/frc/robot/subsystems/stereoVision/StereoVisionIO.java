package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface StereoVisionIO {
  @AutoLog
  public static class StereoVisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  public default void updateInputs(VisionIOInputs inputs) {}
}
