package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;

public class StereoVisionIOPhotonVision implements StereoVisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  /**
   * Creates a new StereoVisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public StereoVisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(StereoVisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    inputs.rayOrigin = robotToCamera.getTranslation(); // ray begins from teh camera

    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.hasGamePiece = true;
        inputs.detectionConfidence = result.getBestTarget().getDetectedObjectConfidence();

        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));

        inputs.rayDirection =
            rayDirection(
                robotToCamera,
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.hasGamePiece = false;
        inputs.detectionConfidence = 0.0;
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }
    }
  }

  static Translation3d rayDirection(Transform3d robotToCamera, Rotation2d yaw, Rotation2d pitch) {
    Rotation3d angleToPiece = new Rotation3d(0.0, pitch.getRadians(), yaw.getRadians());
    return new Translation3d(1.0, 0.0, 0.0) // start of ray infront camera directly
        .rotateBy(angleToPiece) // rotate ray by the angle the camera sees the piece
        .rotateBy(robotToCamera.getRotation()); // determine ray in relation to robot
  }
}
