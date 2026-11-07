package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.angularStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.cameraStdDevFactors;
import static frc.robot.subsystems.vision.VisionConstants.linearStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.maxAmbiguity;
import static frc.robot.subsystems.vision.VisionConstants.maxZError;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameras;

import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.wpilib.command3.Scheduler;
import org.wpilib.epilogue.Logged;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.util.struct.StructSerializable;

@Logged
public class Vision {

  private final VisionConsumer consumer;
  private final PhotonCamera[] cameras;
  private final Alert[] disconnectedAlerts;
  private final List<CameraData> cameraData = new ArrayList<>();
  private final List<Pose3d> allTagPoses;
  private final List<Pose3d> allRobotPoses;
  private final List<Pose3d> allRobotPosesAccepted;
  private final List<Pose3d> allRobotPosesRejected;

  public Vision(VisionConsumer consumer) {
    this.consumer = consumer;

    cameras = new PhotonCamera[VisionConstants.numCameras];
    disconnectedAlerts = new Alert[cameras.length];

    allTagPoses = new ArrayList<>();
    allRobotPoses = new ArrayList<>();
    allRobotPosesAccepted = new ArrayList<>();
    allRobotPosesRejected = new ArrayList<>();

    if (Constants.getRobot() != RobotType.ROBOT_SIMBOT) {
      for (int i = 0; i < VisionConstants.numCameras; i++) {
        cameras[i] = new PhotonCamera(VisionConstants.cameraNames[i]);
        disconnectedAlerts[i] =
            new Alert("Vision Alerts", "Vision camera " + i + " is disconnected.", Level.MEDIUM);
        cameraData.add(new CameraData());
      }

      Scheduler.getDefault().addPeriodic(this::periodic);
    }
  }

  public void periodic() {
    for (int j = 0; j < cameras.length; ++j) {
      var camera = cameras[j];

      cameraData.get(j).connected = camera.isConnected();

      // Read new camera observations
      Set<Short> tagIds = new HashSet<>();
      List<PoseObservation> poseObservations = new LinkedList<>();
      for (var result : camera.getAllUnreadResults()) {
        // Update latest target observation
        if (result.hasTargets()) {
          cameraData.get(j).latestTargetObservation =
              new TargetObservation(
                  Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                  Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
        } else {
          cameraData.get(j).latestTargetObservation = new TargetObservation(new Rotation2d(),
              new Rotation2d());
        }

        // Add pose observation
        if (result.multitagResult.isPresent()) { // Multitag result
          var multitagResult = result.multitagResult.get();

          // Calculate robot pose
          Transform3d fieldToCamera = multitagResult.estimatedPose.best;
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCameras[j].inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          // Calculate average tag distance
          double totalTagDistance = 0.0;
          for (var target : result.targets) {
            totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
          }

          // Add tag IDs
          tagIds.addAll(multitagResult.fiducialIDsUsed);

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robotPose, // 3D pose estimate
                  multitagResult.estimatedPose.ambiguity, // Ambiguity
                  multitagResult.fiducialIDsUsed.size(), // Tag count
                  totalTagDistance / result.targets.size(), // Average tag distance
                  PoseObservationType.PHOTONVISION)); // Observation type

        } else if (!result.targets.isEmpty()) { // Single tag result
          var target = result.targets.getFirst();

          // Calculate robot pose
          var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
          if (tagPose.isPresent()) {
            Transform3d fieldToTarget =
                new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
            Transform3d cameraToTarget = target.bestCameraToTarget;
            Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
            Transform3d fieldToRobot = fieldToCamera.plus(robotToCameras[j].inverse());
            Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(),
                fieldToRobot.getRotation());

            // Add tag ID
            tagIds.add((short) target.fiducialId);

            // Add observation
            poseObservations.add(
                new PoseObservation(
                    result.getTimestampSeconds(), // Timestamp
                    robotPose, // 3D pose estimate
                    target.poseAmbiguity, // Ambiguity
                    1, // Tag count
                    cameraToTarget.getTranslation().getNorm(), // Average tag distance
                    PoseObservationType.PHOTONVISION)); // Observation type
          }
        }
      }

      // Save pose observations to cameraData.get(j) object
      cameraData.get(j).poseObservations = new PoseObservation[poseObservations.size()];
      for (int i = 0; i < poseObservations.size(); i++) {
        cameraData.get(j).poseObservations[i] = poseObservations.get(i);
      }

      // Save tag IDs to cameraData.get(j) objects
      cameraData.get(j).tagIds = new int[tagIds.size()];
      int i = 0;
      for (int id : tagIds) {
        cameraData.get(j).tagIds[i++] = id;
      }
    }

    boolean anyAcceptedObservation = false;
    allTagPoses.clear();
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    allRobotPosesRejected.clear();

    // Loop over cameras
    for (int i = 0; i < cameras.length; i++) {
      // Update disconnected alert
      disconnectedAlerts[i].set(!cameraData.get(i).connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : cameraData.get(i).tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      // Loop over pose observations
      for (var observation : cameraData.get(i).poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }
        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (i < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[i];
          angularStdDev *= cameraStdDevFactors[i];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    RobotState.getInstance().PoseConfidence = !allRobotPosesAccepted.isEmpty();
  }

  public enum PoseObservationType {
    PHOTONVISION
  }

  /**
   * This functional interface is used in vision processing pipelines to handle incoming data for
   * updating the robot's state with vision-based information.
   */
  @FunctionalInterface
  public interface VisionConsumer {

    /**
     * Processes incoming vision data to update the robot's state using vision-based measurements.
     *
     * @param visionRobotPoseMeters    The measured robot pose in meters
     * @param timestampSeconds         The timestamp, in seconds, at which the vision measurement
     *                                 was taken.
     * @param visionMeasurementStdDevs The confidence of the measurement in terms of position and
     *                                 rotation.
     */
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  /**
   * Represents the angle to a simple target, not used for pose estimation.
   */
  public record TargetObservation(Rotation2d tx, Rotation2d ty) {

  }

  /**
   * Represents a robot pose sample used for pose estimation.
   */
  public record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {

  }

  @Logged
  public static class CameraData implements StructSerializable {

    public boolean connected;
    public TargetObservation latestTargetObservation;
    public PoseObservation[] poseObservations;
    public int[] tagIds;
  }
}
