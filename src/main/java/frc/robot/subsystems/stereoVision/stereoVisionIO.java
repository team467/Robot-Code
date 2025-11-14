package frc.robot.subsystems.stereoVision;

import java.awt.geom.Point2D;
import org.littletonrobotics.junction.AutoLog;

public interface stereoVisionIO {
  @AutoLog
  public static class stereoVisionInputs {
    public boolean[] connected = new boolean[] {false, false};
    public PoseObservation[] poseObservations = new PoseObservation[] {};
    public boolean seesGamePiece = false;
  }

  public static record PoseObservation(
      Point2D camera1Center, Point2D camera2Center, double distance, gamePieceType type) {}

  public static enum gamePieceType {
    CORAL,
    ALGAE
  }

  public default void updateInputs(stereoVisionInputs inputs) {}
}
