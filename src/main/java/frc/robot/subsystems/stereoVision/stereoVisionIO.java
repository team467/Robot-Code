package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Transform2d;
import org.littletonrobotics.junction.AutoLog;

public interface stereoVisionIO {
  @AutoLog
  public static class stereoVisionInputs {
    public boolean[] connected = new boolean[] {false, false};
    public PoseObservation[] poseObservations = new PoseObservation[] {};
    public boolean seesGamePiece = false;
  }

  public static record PoseObservation(Transform2d pose, gamePieceType type) {}

  public static enum gamePieceType {
    CORAL,
    ALGAE,
    NULL
  }

  public default void updateInputs(stereoVisionInputs inputs) {}
}
