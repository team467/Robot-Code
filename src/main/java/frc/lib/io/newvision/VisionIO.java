package frc.lib.io.newvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;

public interface VisionIO {
  class VisionIOInputs implements LoggableInputs {
    public byte[] rawEntry = new byte[] {};
    public Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

    @Override
    public void toLog(LogTable table) {
      table.put("RawEntry", rawEntry);
      table.put("EstimatedPose/Present", estimatedPose.isPresent());
      if (estimatedPose.isPresent()) {
        double[] serializedEstimatedPose = new double[7];
        serializedEstimatedPose[0] = estimatedPose.get().estimatedPose.getTranslation().getX();
        serializedEstimatedPose[1] = estimatedPose.get().estimatedPose.getTranslation().getY();
        serializedEstimatedPose[2] = estimatedPose.get().estimatedPose.getTranslation().getZ();
        serializedEstimatedPose[3] =
            estimatedPose.get().estimatedPose.getRotation().getQuaternion().getW();
        serializedEstimatedPose[4] =
            estimatedPose.get().estimatedPose.getRotation().getQuaternion().getX();
        serializedEstimatedPose[5] =
            estimatedPose.get().estimatedPose.getRotation().getQuaternion().getY();
        serializedEstimatedPose[6] =
            estimatedPose.get().estimatedPose.getRotation().getQuaternion().getZ();
        table.put("EstimatedPose/Pose", serializedEstimatedPose);

        Pose2d estimatedPose2d = estimatedPose.get().estimatedPose.toPose2d();
        double[] serializedEstimatedPose2d = new double[3];
        serializedEstimatedPose2d[0] = estimatedPose2d.getX();
        serializedEstimatedPose2d[1] = estimatedPose2d.getY();
        serializedEstimatedPose2d[2] = estimatedPose2d.getRotation().getRadians();
        table.put("EstimatedPose/Pose2d", serializedEstimatedPose2d);
        table.put("EstimatedPose/TimestampSeconds", estimatedPose.get().timestampSeconds);
      }
    }

    @Override
    public void fromLog(LogTable table) {
      rawEntry = table.get("RawEntry").getRaw();
      if (table.get("EstimatedPose/Present").getBoolean()) {
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
                table.get("EstimatedPose/TimestampSeconds").getDouble(),
                null);
        this.estimatedPose = Optional.of(estimatedPose);
      } else {
        this.estimatedPose = Optional.empty();
      }
    }
  }

  default void updateInputs(VisionIOInputs inputs) {}
}
