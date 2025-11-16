package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import org.littletonrobotics.junction.AutoLog;

public interface stereoVisionIO {
  @AutoLog
  public static class stereoVisionInputs {
    public boolean[] connected = new boolean[] {false, false};
    public objectObservation[] objectObservations = new objectObservation[] {};
    public poseObservation[] poseObservations = new poseObservation[] {};
    public boolean seesGamePiece = false;
  }

  public static record objectObservation(Transform2d pose, gamePieceType type) {}

  public static record poseObservation(Pose2d pose, gamePieceType type) {}

  public static enum gamePieceType {
    CORAL,
    ALGAE,
    NULL
  }

  public default void updateInputs(stereoVisionInputs inputs) {}
}
