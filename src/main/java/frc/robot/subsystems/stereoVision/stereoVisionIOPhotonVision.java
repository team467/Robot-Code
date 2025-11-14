package frc.robot.subsystems.stereoVision;

import java.util.LinkedList;
import java.util.List;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;

public class stereoVisionIOPhotonVision implements stereoVisionIO {

  protected final PhotonCamera camera1;
  protected final PhotonCamera camera2;


  public stereoVisionIOPhotonVision(
      String name1, String name2) {
    camera1 = new PhotonCamera(name1);
    camera2 = new PhotonCamera(name2);
  }

  @Override
  public void updateInputs(stereoVisionIOInputs inputs) {
    inputs.connected[0] = camera1.isConnected();
    inputs.connected[1] = camera2.isConnected();
    List<PoseObservation> poseObservations = new LinkedList<>();
    var results1 = camera1.getAllUnreadResults();
    var results2 = camera2.getAllUnreadResults();
    if (!results1.isEmpty() && !results2.isEmpty()) {
      int i = 0;
      inputs.seesGamePiece = true;
      while (results1.get(i).hasTargets() && results2.get(i).hasTargets()) {
        var target1 = results1.get(i).getBestTarget();
        var target2 = results2.get(i).getBestTarget();
        Pair<Double,Double> center1 = new Pair((target1.detectedCorners.get(0).y - target1.detectedCorners.get(3).y)/2, (target1.detectedCorners.get(2).x - target1.detectedCorners.get(3).x)/2);
        Pair<Double,Double> center2 = new Pair((target2.detectedCorners.get(0).y - target2.detectedCorners.get(3).y)/2, (target2.detectedCorners.get(2).x - target2.detectedCorners.get(3).x)/2);
        var finalType= gamePieceType.NULL;
        if (target1.getDetectedObjectClassID() == 1){
          finalType = gamePieceType.CORAL;
        } else {
          finalType = gamePieceType.ALGAE;
        }
        double disparity = (Math.abs(center1.getSecond() - center2.getSecond()));
        double depth = (stereoVisionConstants.focalLength * stereoVisionConstants.baseLength)/disparity;
        double x  = (((center1.getSecond()+center2.getSecond())/2-320) * depth) / stereoVisionConstants.focalLength;
        double y = Math.sqrt((depth*depth) - (stereoVisionConstants.cameraHeight*stereoVisionConstants.cameraHeight) - (x*x));
        double theta =Math.atan2(y,x);
        Transform2d tranformation = new Transform2d(
            new Translation2d(x, y), new Rotation2d(theta));
        poseObservations.add(new PoseObservation(tranformation, finalType));
      }
    } else {
      inputs.seesGamePiece = false;
      inputs.poseObservations = new PoseObservation[] {};
    }
  }
}
