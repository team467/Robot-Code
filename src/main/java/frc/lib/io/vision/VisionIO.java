package frc.lib.io.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public interface VisionIO {
  class VisionIOInputs implements LoggableInputs {

    /** A list with a size of 8 containing the targets that the vision system has detected. */
    public List<Optional<PhotonTrackedTarget>> targets = List.of(Optional.empty());
    /** The latency of the vision system in milliseconds. */
    public double latencyMillis = -1;
    /** The timestamp of the vision system in milliseconds. */
    public double timestampSeconds = -1;
    /** Returns whether the vision system has detected any targets. */
    public boolean hasTargets = false;
    /** The estimated pose of the robot. */
    public Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

    @Override
    public void toLog(LogTable table) {
      for (int i = 0; i < 7; i++) {
        PhotonTrackedTarget target;
        if (i < targets.size()) {
          target = targets.get(i).orElse(null);
        } else {
          target = null;
        }
        if (target != null) {
          table.put("Target/" + i + "/Valid", true);
          table.put("Target/" + i + "/Yaw", target.getYaw());
          table.put("Target/" + i + "/Pitch", target.getPitch());
          table.put("Target/" + i + "/Area", target.getArea());
          table.put("Target/" + i + "/Skew", target.getSkew());
          table.put("Target/" + i + "/FiducialId", target.getFiducialId());
          double[] serializedBestCameraToTarget = new double[7];
          serializedBestCameraToTarget[0] = target.getBestCameraToTarget().getTranslation().getX();
          serializedBestCameraToTarget[1] = target.getBestCameraToTarget().getTranslation().getY();
          serializedBestCameraToTarget[2] = target.getBestCameraToTarget().getTranslation().getZ();
          serializedBestCameraToTarget[3] =
              target.getBestCameraToTarget().getRotation().getQuaternion().getW();
          serializedBestCameraToTarget[4] =
              target.getBestCameraToTarget().getRotation().getQuaternion().getX();
          serializedBestCameraToTarget[5] =
              target.getBestCameraToTarget().getRotation().getQuaternion().getY();
          serializedBestCameraToTarget[6] =
              target.getBestCameraToTarget().getRotation().getQuaternion().getZ();
          table.put("Target/" + i + "/BestCameraToTarget", serializedBestCameraToTarget);
          double[] serializedAltCameraToTarget = new double[7];
          serializedAltCameraToTarget[0] =
              target.getAlternateCameraToTarget().getTranslation().getX();
          serializedAltCameraToTarget[1] =
              target.getAlternateCameraToTarget().getTranslation().getY();
          serializedAltCameraToTarget[2] =
              target.getAlternateCameraToTarget().getTranslation().getZ();
          serializedAltCameraToTarget[3] =
              target.getAlternateCameraToTarget().getRotation().getQuaternion().getW();
          serializedAltCameraToTarget[4] =
              target.getAlternateCameraToTarget().getRotation().getQuaternion().getX();
          serializedAltCameraToTarget[5] =
              target.getAlternateCameraToTarget().getRotation().getQuaternion().getY();
          serializedAltCameraToTarget[6] =
              target.getAlternateCameraToTarget().getRotation().getQuaternion().getZ();
          table.put("Target/" + i + "/AltCameraToTarget", serializedAltCameraToTarget);
          table.put("Target/" + i + "/PoseAmbiguity", target.getPoseAmbiguity());
          for (int j = 0; j < 4; j++) {
            table.put(
                "Target/" + i + "/MinAreaRectCorners/" + j + "/X",
                target.getMinAreaRectCorners().get(j).x);
            table.put(
                "Target/" + i + "/MinAreaRectCorners/" + j + "/Y",
                target.getMinAreaRectCorners().get(j).y);
            table.put(
                "Target/" + i + "/DetectedCorners/" + j + "/X",
                target.getDetectedCorners().get(j).x);
            table.put(
                "Target/" + i + "/DetectedCorners/" + j + "/Y",
                target.getDetectedCorners().get(j).y);
          }
        } else {
          table.put("Target/" + i + "/Valid", false);
        }
      }
      table.put("HasTargets", hasTargets);
      table.put("LatencyMillis", latencyMillis);
      table.put("TimestampSeconds", timestampSeconds);
      EstimatedRobotPose estimatedPose = this.estimatedPose.orElse(null);
      if (estimatedPose != null) {
        double[] serializedEstimatedPose = new double[7];
        serializedEstimatedPose[0] = estimatedPose.estimatedPose.getTranslation().getX();
        serializedEstimatedPose[1] = estimatedPose.estimatedPose.getTranslation().getY();
        serializedEstimatedPose[2] = estimatedPose.estimatedPose.getTranslation().getZ();
        serializedEstimatedPose[3] =
            estimatedPose.estimatedPose.getRotation().getQuaternion().getW();
        serializedEstimatedPose[4] =
            estimatedPose.estimatedPose.getRotation().getQuaternion().getX();
        serializedEstimatedPose[5] =
            estimatedPose.estimatedPose.getRotation().getQuaternion().getY();
        serializedEstimatedPose[6] =
            estimatedPose.estimatedPose.getRotation().getQuaternion().getZ();
        table.put("EstimatedPose/Valid", true);
        table.put("EstimatedPose/Pose", serializedEstimatedPose);
        table.put("EstimatedPose/TimestampSeconds", estimatedPose.timestampSeconds);
      } else {
        table.put("EstimatedPose/Valid", false);
      }
    }

    @Override
    public void fromLog(LogTable table) {
      for (int i = 0; i < 7; i++) {
        if (table.get("Target/" + i + "/Valid").getBoolean()) {
          PhotonTrackedTarget target =
              new PhotonTrackedTarget(
                  table.get("Target/" + i + "/Yaw").getDouble(),
                  table.get("Target/" + i + "/Pitch").getDouble(),
                  table.get("Target/" + i + "/Area").getDouble(),
                  table.get("Target/" + i + "/Skew").getDouble(),
                  ((int) table.get("Target/" + i + "/FiducialId").getInteger()),
                  new Transform3d(
                      new Translation3d(
                          table.get("Target/" + i + "/BestCameraToTarget").getDoubleArray()[0],
                          table.get("Target/" + i + "/BestCameraToTarget").getDoubleArray()[1],
                          table.get("Target/" + i + "/BestCameraToTarget").getDoubleArray()[2]),
                      new Rotation3d(
                          new Quaternion(
                              table.get("Target/" + i + "/BestCameraToTarget").getDoubleArray()[3],
                              table.get("Target/" + i + "/BestCameraToTarget").getDoubleArray()[4],
                              table.get("Target/" + i + "/BestCameraToTarget").getDoubleArray()[5],
                              table.get("Target/" + i + "/BestCameraToTarget")
                                  .getDoubleArray()[6]))),
                  new Transform3d(
                      new Translation3d(
                          table.get("Target/" + i + "/AltCameraToTarget").getDoubleArray()[0],
                          table.get("Target/" + i + "/AltCameraToTarget").getDoubleArray()[1],
                          table.get("Target/" + i + "/AltCameraToTarget").getDoubleArray()[2]),
                      new Rotation3d(
                          new Quaternion(
                              table.get("Target/" + i + "/AltCameraToTarget").getDoubleArray()[3],
                              table.get("Target/" + i + "/AltCameraToTarget").getDoubleArray()[4],
                              table.get("Target/" + i + "/AltCameraToTarget").getDoubleArray()[5],
                              table.get("Target/" + i + "/AltCameraToTarget")
                                  .getDoubleArray()[6]))),
                  table.get("Target/" + i + "/PoseAmbiguity").getDouble(),
                  List.of(
                      new TargetCorner(
                          table.get("Target/" + i + "/MinAreaRectCorners/0/X").getDouble(),
                          table.get("Target/" + i + "/MinAreaRectCorners/0/Y").getDouble()),
                      new TargetCorner(
                          table.get("Target/" + i + "/MinAreaRectCorners/1/X").getDouble(),
                          table.get("Target/" + i + "/MinAreaRectCorners/1/Y").getDouble()),
                      new TargetCorner(
                          table.get("Target/" + i + "/MinAreaRectCorners/2/X").getDouble(),
                          table.get("Target/" + i + "/MinAreaRectCorners/2/Y").getDouble()),
                      new TargetCorner(
                          table.get("Target/" + i + "/MinAreaRectCorners/3/X").getDouble(),
                          table.get("Target/" + i + "/MinAreaRectCorners/3/Y").getDouble())),
                  List.of(
                      new TargetCorner(
                          table.get("Target/" + i + "/DetectedCorners/0/X").getDouble(),
                          table.get("Target/" + i + "/DetectedCorners/0/Y").getDouble()),
                      new TargetCorner(
                          table.get("Target/" + i + "/DetectedCorners/1/X").getDouble(),
                          table.get("Target/" + i + "/DetectedCorners/1/Y").getDouble()),
                      new TargetCorner(
                          table.get("Target/" + i + "/DetectedCorners/2/X").getDouble(),
                          table.get("Target/" + i + "/DetectedCorners/2/Y").getDouble()),
                      new TargetCorner(
                          table.get("Target/" + i + "/DetectedCorners/3/X").getDouble(),
                          table.get("Target/" + i + "/DetectedCorners/3/Y").getDouble())));
          targets.set(i, Optional.of(target));
        } else {
          targets.set(i, Optional.empty());
        }
      }
      hasTargets = table.get("HasTargets").getBoolean();
      latencyMillis = table.get("LatencyMillis").getDouble();
      timestampSeconds = table.get("TimestampSeconds").getDouble();
      if (table.get("EstimatedPose/Valid").getBoolean()) {
        double[] serializedEstimatedPose = table.get("EstimatedPose/Pose").getDoubleArray();
        EstimatedRobotPose estimatedPose =
            new EstimatedRobotPose(
                new Pose3d(
                    new Translation3d(
                        serializedEstimatedPose[0],
                        serializedEstimatedPose[1],
                        serializedEstimatedPose[2]),
                    new Rotation3d(
                        new Quaternion(
                            serializedEstimatedPose[3],
                            serializedEstimatedPose[4],
                            serializedEstimatedPose[5],
                            serializedEstimatedPose[6]))),
                table.get("EstimatedPose/TimestampSeconds").getDouble());
        this.estimatedPose = Optional.of(estimatedPose);
      } else {
        this.estimatedPose = Optional.empty();
      }
    }
  }

  default void updateInputs(VisionIOInputs inputs) {}

  default void setDriverMode(boolean driverMode) {}

  default void setPipelineIndex(int index) {}
}
