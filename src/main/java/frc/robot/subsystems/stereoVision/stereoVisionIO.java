package frc.robot.subsystems.stereoVision;

import org.littletonrobotics.junction.AutoLog;

public interface stereoVisionIO {
  @AutoLog
  public static class stereoVisionIOInputs {
    public boolean[] connected = new boolean[] {false, false};
    public PoseObservation[] poseObservations = new PoseObservation[] {};
    public boolean seesGamePiece = false;
  }

  public static record PoseObservation(
      double camera1Center, double camera2Center, double distance, gamePieceType type) {}

  public static enum gamePieceType {
    CORAL,
    ALGAE
  }

  public default void updateInputs(stereoVisionIOInputs inputs) {}
}
