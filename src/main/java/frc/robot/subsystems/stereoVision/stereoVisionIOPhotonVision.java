package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.LinkedList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class stereoVisionIOPhotonVision implements stereoVisionIO {

  protected final PhotonCamera camera1;
  protected final PhotonCamera camera2;
  private final Trigger trigger;

  public stereoVisionIOPhotonVision(String name1, String name2) {
    camera1 = new PhotonCamera(name1);
    camera2 = new PhotonCamera(name2);
    trigger =
        new Trigger(
                () -> {
                  var results1 = !camera1.getAllUnreadResults().isEmpty();
                  var results2 = !camera2.getAllUnreadResults().isEmpty();
                  return results1;
                })
            .debounce(0.01);
  }

  @Override
  public void updateInputs(stereoVisionInputs inputs) {
    inputs.connected[0] = camera1.isConnected();
    inputs.connected[1] = camera2.isConnected();
    List<objectObservation> objectObservations = new LinkedList<>();
    var results1raw = camera1.getAllUnreadResults();
    var results2raw = camera2.getAllUnreadResults();
    Pair<List<PhotonPipelineResult>, List<PhotonPipelineResult>> unreadResults =
        new Pair<>(results1raw, results2raw);
    Pair<List<PhotonTrackedTarget>, List<PhotonTrackedTarget>> sortedResults =
        sortUnreadResults(unreadResults);
    var results1 = sortedResults.getFirst();
    var results2 = sortedResults.getSecond();

    // change this later
    if (!results1.isEmpty() && !results2.isEmpty()) {
      for (int i = 0; i < results1.size(); i++) {
        var target1 = results1.get(i);
        var target2 = results2.get(i);
        Pair<Double, Double> center1 =
            new Pair<>(
                (target1.detectedCorners.get(0).y + target1.detectedCorners.get(3).y) / 2,
                (target1.detectedCorners.get(2).x + target1.detectedCorners.get(3).x) / 2);
        Pair<Double, Double> center2 =
            new Pair<>(
                (target2.detectedCorners.get(0).y + target2.detectedCorners.get(3).y) / 2,
                (target2.detectedCorners.get(2).x + target2.detectedCorners.get(3).x) / 2);
        var finalType = gamePieceType.NULL;
        if (target1.getDetectedObjectClassID() == 1) {
          finalType = gamePieceType.CORAL;
        } else {
          finalType = gamePieceType.ALGAE;
        }
        Transform2d transformation =
            getTransform2d(center1, center2).plus(stereoVisionConstants.toRobotCenter.inverse());
        objectObservations.add(new objectObservation(transformation, finalType));
        i++;
      }
      inputs.objectObservations = objectObservations.toArray(new objectObservation[0]);
    } else {
      inputs.objectObservations = new objectObservation[] {};
    }

    inputs.seesGamePiece = trigger.getAsBoolean();
  }

  private static Transform2d getTransform2d(
      Pair<Double, Double> center1, Pair<Double, Double> center2) {
    double disparity = (Math.abs(center1.getSecond() - center2.getSecond()));
    double depth =
        (stereoVisionConstants.focalLength * stereoVisionConstants.baseLength) / disparity;
    double x =
        (((center1.getSecond() + center2.getSecond()) / 2 - 320) * depth)
            / stereoVisionConstants.focalLength;
    double y =
        Math.sqrt(
            (depth * depth)
                - (stereoVisionConstants.cameraHeight * stereoVisionConstants.cameraHeight)
                - (x * x));
    double theta = Math.atan2(y, x);
    return new Transform2d(new Translation2d(x, y), new Rotation2d(theta));
  }
  // TODO: Test this when detecting multiple coral
  private static Pair<List<PhotonTrackedTarget>, List<PhotonTrackedTarget>> sortUnreadResults(
      Pair<List<PhotonPipelineResult>, List<PhotonPipelineResult>> unreadResults) {
    List<PhotonTrackedTarget> sortedTargets1 = new LinkedList<>();
    List<PhotonTrackedTarget> sortedTargets2 = new LinkedList<>();
    int i = 1;
    for (var target1 : unreadResults.getFirst()) {
      PhotonPipelineResult lowestDisparity = null;
      double lowestDisparityValue = Double.MAX_VALUE;
      if (i <= unreadResults.getSecond().size()) {
        for (var target2 : unreadResults.getSecond()) {
          double disparity =
              ((target1.getBestTarget().detectedCorners.get(2).x
                          + target1.getBestTarget().detectedCorners.get(3).x)
                      / 2)
                  - ((target2.getBestTarget().detectedCorners.get(2).x
                          + target2.getBestTarget().detectedCorners.get(3).x)
                      / 2);
          if (disparity < lowestDisparityValue) {
            lowestDisparityValue = disparity;
            lowestDisparity = target2;
          }
        }
      } else {
        return new Pair<>(sortedTargets1, sortedTargets2);
      }
      if (lowestDisparity.getBestTarget().objDetectConf < 0.5
          || target1.getBestTarget().objDetectConf < 0.5) {
        continue;
      } else {
        sortedTargets1.add(target1.getBestTarget());
        sortedTargets2.add(lowestDisparity.getBestTarget());
      }

      i++;
    }
    return new Pair<>(sortedTargets1, sortedTargets2);
  }
}
