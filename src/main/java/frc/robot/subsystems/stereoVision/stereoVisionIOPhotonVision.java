package frc.robot.subsystems.stereoVision;

import java.awt.geom.Point2D;
import java.util.LinkedList;
import java.util.List;
import org.photonvision.PhotonCamera;

public class stereoVisionIOPhotonVision implements stereoVisionIO {

  protected final PhotonCamera camera1;
  protected final PhotonCamera camera2;
  protected final double pupillaryDistance;
  protected final double cameraHeight;

  public stereoVisionIOPhotonVision(
      String name1, String name2, double pupillaryDistance, double cameraHeight) {
    camera1 = new PhotonCamera(name1);
    camera2 = new PhotonCamera(name2);
    this.pupillaryDistance = pupillaryDistance;
    this.cameraHeight = cameraHeight;
  }

  @Override
  public void updateInputs(stereoVisionInputs inputs) {
    inputs.connected[0] = camera1.isConnected();
    inputs.connected[1] = camera2.isConnected();
    List<PoseObservation> poseObservations = new LinkedList<>();
    var results1 = camera1.getAllUnreadResults();
    var results2 = camera2.getAllUnreadResults();
    if (!results1.isEmpty() && !results2.isEmpty()) {
      int i = 0;
      while (results1.get(i).hasTargets() && results2.get(i).hasTargets()) {
        var target1 = results1.get(i).getBestTarget();
        var target2 = results2.get(i).getBestTarget();
        Point2D center1 = new Point2D.Double(
            ((target1.detectedCorners.get(0).y - target1.detectedCorners.get(3).y) / 2),
            ((target1.detectedCorners.get(2).x - target1.detectedCorners.get(3).x) / 2)
        );
        Point2D center2 = new Point2D.Double(
          ((target2.detectedCorners.get(0).y - target2.detectedCorners.get(3).y) / 2),
          ((target2.detectedCorners.get(2).x - target2.detectedCorners.get(3).x) / 2)
        );
        var type = target1.getDetectedObjectClassID();
      }
    } else {
      inputs.seesGamePiece = false;
      inputs.poseObservations = new PoseObservation[] {};
    }
  }
}
