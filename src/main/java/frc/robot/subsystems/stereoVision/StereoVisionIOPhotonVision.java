package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import org.photonvision.PhotonCamera;

public class StereoVisionIOPhotonVision implements StereoVisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */

  public StereoVisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  public void updates(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new VisionIO.TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new VisionIO.TargetObservation(new Rotation2d(),
            new Rotation2d());
      }
    }


  }
}
