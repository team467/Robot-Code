package frc.robot.subsystems.stereoVision;

import static frc.robot.subsystems.stereoVision.StereoVisionConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class StereoVision extends SubsystemBase {
  private final StereoVisionIO[] io;
  private final StereoVisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private Translation3d lastGamePiecePosition = null;

  public StereoVision(StereoVisionIO... io) {
    this.io = io;

    // Initialize inputs
    this.inputs = new StereoVisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new StereoVisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Stereo Vision camera " + Integer.toString(i) + " is disconnected.",
              AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target (game piece), which can be used for simple servoing with
   * vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  public Translation3d getLastGamePiecePosition() {
    return lastGamePiecePosition;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("StereoVision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
    }

    // performs triangulations calcualtions if both cameras are connected see the fuel and are
    // confident
    lastGamePiecePosition = null;
    if (inputs[0].connected
        && inputs[1].connected
        && inputs[0].hasGamePiece
        && inputs[1].hasGamePiece
        && inputs[0].detectionConfidence >= minDetectionConfidence
        && inputs[1].detectionConfidence >= minDetectionConfidence) {
      lastGamePiecePosition =
          triangulate(
              inputs[0].rayOrigin, inputs[0].rayDirection,
              inputs[1].rayOrigin, inputs[1].rayDirection);
      Logger.recordOutput("StereoVision/LastGamePiecePosition", lastGamePiecePosition);
    }
  }

  /** Finds the game piece's position from the two cameras rays. */
  static Translation3d triangulate(
      Translation3d rayOriginA,
      Translation3d rayDirectionA,
      Translation3d rayOriginB,
      Translation3d rayDirectionB) {

    // determine slopes of btoh rays
    double slopeA = rayDirectionA.getY() / rayDirectionA.getX();
    double slopeB = rayDirectionB.getY() / rayDirectionB.getX();

    //  x pos/distacne forward where the rays have the same y pos/ distance sideways
    double x =
        (rayOriginB.getY()
                - rayOriginA.getY()
                + rayOriginA.getX() * slopeA
                - rayOriginB.getX() * slopeB)
            / (slopeA - slopeB);

    // y pos where rays intersect
    double y = rayOriginA.getY() + (x - rayOriginA.getX()) * slopeA;

    // doesnt return hieght
    return new Translation3d(x, y, 0.0);
  }
}
