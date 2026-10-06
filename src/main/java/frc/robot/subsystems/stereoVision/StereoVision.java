package frc.robot.subsystems.stereoVision;

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
  }

  /** Finds the game piece's 3D position from the two cameras rays. */
  static Translation3d triangulate() {
    // TODO: implement eq to find the point where the two cams rays intersect
    return null;
  }
}
