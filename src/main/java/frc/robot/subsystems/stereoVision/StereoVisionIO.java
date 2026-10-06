package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.AutoLog;

public interface StereoVisionIO {
  @AutoLog
  public static class StereoVisionIOInputs {
    public boolean connected = false;
    public boolean hasGamePiece = false;
    public double detectionConfidence = 0.0; // for game piece

    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());

    public Translation3d rayOrigin = new Translation3d();
    public Translation3d rayDirection = new Translation3d();
  }

  /**
   * Represents the angle to a simple target (the game piece), not used for pose estimation.
   *
   * @param tx -> yaw
   * @param ty -> pitch
   */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  public default void updateInputs(StereoVisionIOInputs inputs) {}
}
