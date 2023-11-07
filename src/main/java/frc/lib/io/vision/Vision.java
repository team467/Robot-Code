package frc.lib.io.vision;

import static frc.lib.io.vision.VisionConstants.MAX_POSE_DIFFERENCE_METERS;
import static frc.lib.io.vision.VisionConstants.POSE_DIFFERENCE_THRESHOLD_METERS;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.vision.VisionIO.VisionIOInputs;
import frc.lib.utils.RobotOdometry;
import frc.lib.utils.TunableNumber;
import frc.robot.FieldConstants;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The Vision subsystem is responsible for updating the robot's estimated pose based on a collection
 * of cameras capturing AprilTags. The Vision subsystem comprises multiple VisionIO objects, each of
 * which is responsible for producing a single PhotonPipelineResult. There is a one-to-one
 * relationship between each VisionIO object and each coprocessor (e.g., Raspberry Pi) running
 * PhotonVision.
 */
public class Vision extends SubsystemBase {
  private final List<VisionIO> visionIOs;
  private final List<Transform3d> camerasToRobots;
  private final VisionIOInputs[] inputs;
  private final double[] lastTimestamps;

  private final AprilTagFieldLayout layout;

  private boolean isEnabled = true;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final TunableNumber poseDifferenceThreshold =
      new TunableNumber("Vision/VisionPoseThreshold", POSE_DIFFERENCE_THRESHOLD_METERS);
  private final TunableNumber stdDevSlope = new TunableNumber("Vision/stdDevSlope", 0.10);
  private final TunableNumber stdDevPower = new TunableNumber("Vision/stdDevPower", 2.0);

  private static class RobotPoseFromAprilTag {
    public final Pose3d robotPose;
    public final double distanceToAprilTag;

    public RobotPoseFromAprilTag(Pose3d robotPose, double distance) {
      this.robotPose = robotPose;
      this.distanceToAprilTag = distance;
    }
  }

  /**
   * Create a new Vision subsystem. The number of VisionIO objects passed to the constructor must
   * match the number of robot-to-camera transforms returned by the RobotConfig singleton.
   *
   * @param visionIOs One or more VisionIO objects, each of which is responsible for producing a
   *     single PhotonPipelineResult. There is a one-to-one relationship between each VisionIO
   *     object and each coprocessor (e.g., Raspberry Pi) running PhotonVision.
   */
  public Vision(List<VisionIO> visionIOs, List<Transform3d> camerasToRobots) {
    this.visionIOs = visionIOs;
    this.camerasToRobots = camerasToRobots;
    this.lastTimestamps = new double[visionIOs.size()];
    this.inputs = new VisionIOInputs[visionIOs.size()];
    for (int i = 0; i < visionIOs.size(); i++) {
      this.inputs[i] = new VisionIOInputs();
    }

    // retrieve a reference to the pose estimator singleton
    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();

    // load and log all the AprilTags in the field layout file
    layout = FieldConstants.aprilTags;

    for (AprilTag tag : layout.getTags()) {
      Logger.recordOutput("Vision/AprilTags/" + tag.ID, tag.pose);
    }
  }

  /**
   * This method is invoked each iteration of the scheduler. It updates the inputs for each of the
   * VisionIO objects and, for each, updates the pose estimator based on the most recent detected
   * AprilTags.
   */
  @Override
  public void periodic() {
    Logger.recordOutput("Vision/IsUpdating", true);
    for (int i = 0; i < visionIOs.size(); i++) {
      visionIOs.get(i).updateInputs(inputs[i]);
      Logger.processInputs("Vision" + i, inputs[i]);

      // only process the vision data if the timestamp is newer than the last one
      if (lastTimestamps[i] < inputs[i].lastTimestamp) {
        lastTimestamps[i] = inputs[i].lastTimestamp;
        RobotPoseFromAprilTag poseAndDistance = getRobotPoseOptimized(i);
        Pose3d robotPose = poseAndDistance.robotPose;

        Logger.recordOutput("Vision/ValidTarget", robotPose != null);
        if (robotPose == null) return;

        // only update the pose estimator if the pose from the vision data is close to the estimated
        // robot pose
        if (poseEstimator
                .getEstimatedPosition()
                .minus(robotPose.toPose2d())
                .getTranslation()
                .getNorm()
            < MAX_POSE_DIFFERENCE_METERS) {

          // only update the pose estimator if the vision subsystem is enabled
          if (isEnabled) {
            // when updating the pose estimator, specify standard deviations based on the distance
            // from the robot to the AprilTag (the greater the distance, the less confident we are
            // in the measurement)
            poseEstimator.addVisionMeasurement(
                robotPose.toPose2d(),
                inputs[i].lastTimestamp,
                getStandardDeviations(poseAndDistance.distanceToAprilTag));
            Logger.recordOutput("Vision/IsUpdating", true);
          }

          Logger.recordOutput("Vision/RobotPose" + i, robotPose.toPose2d());
          Logger.recordOutput("Vision/IsEnabled", isEnabled);
        }
      }
    }
  }

  /**
   * Returns true if the vision subsystem is enabled.
   *
   * @return true if the vision subsystem is enabled
   */
  public boolean isEnabled() {
    return isEnabled;
  }

  /**
   * Retrieves the best estimated pose of the robot based on data from the vision subsystem.
   *
   * @return the best robot pose if available, or null if no pose is found
   */
  public Pose3d getBestRobotPose() {
    Pose3d robotPoseFromClosestTarget = null;
    double closestTargetDistance = Double.MAX_VALUE;
    for (int i = 0; i < visionIOs.size(); i++) {
      RobotPoseFromAprilTag poseAndDistance = getRobotPoseOptimized(i);
      Pose3d robotPose = poseAndDistance.robotPose;
      double distanceToAprilTag = poseAndDistance.distanceToAprilTag;
      if (robotPose != null && distanceToAprilTag < closestTargetDistance) {
        robotPoseFromClosestTarget = robotPose;
        closestTargetDistance = distanceToAprilTag;
      }
    }
    return robotPoseFromClosestTarget;
  }

  /**
   * Enable or disable the vision subsystem.
   *
   * @param enable enables the vision subsystem if true; disables if false
   */
  public void enable(boolean enable) {
    isEnabled = enable;
  }

  /**
   * Returns true if the robot's pose based on vision data is within the specified threshold of the
   * robot's pose based on the pose estimator. This method can be used to trigger a transition from
   * driver control to automated control once confident that the estimated pose is accurate.
   *
   * @return true if the robot's pose based on vision data is within the specified threshold of the
   *     robot's pose based on the pose estimator
   */
  public boolean posesHaveConverged() {
    for (int i = 0; i < visionIOs.size(); i++) {
      Pose3d robotPose = getRobotPoseOptimized(i).robotPose;
      if (robotPose != null
          && poseEstimator
                  .getEstimatedPosition()
                  .minus(robotPose.toPose2d())
                  .getTranslation()
                  .getNorm()
              < poseDifferenceThreshold.get()) {
        Logger.recordOutput("Vision/posesInLine", true);
        return true;
      }
    }
    Logger.recordOutput("Vision/posesInLine", false);
    return false;
  }

  /**
   * Returns a matrix representing the standard deviations of the vision measurement. The standard
   * deviations are calculated based on the distance from the robot to the target and can be
   * adjusted using tuning parameters.
   *
   * @param targetDistance the distance from the robot to the target
   * @return a matrix representing the standard deviations of the vision measurement
   */
  private Matrix<N3, N1> getStandardDeviations(double targetDistance) {
    // the standard deviation of the vision measurement is a function of the distance from the robot
    // to the AprilTag and can be tuned
    double stdDevTrust = stdDevSlope.get() * (Math.pow(targetDistance, stdDevPower.get()));
    return VecBuilder.fill(stdDevTrust, stdDevTrust, stdDevTrust);
  }

  /**
   * Returns the robot pose based on vision data and distance to the AprilTag that is closest to the
   * robot. The current algorithm simply uses the AprilTag that is closest to the robot from which
   * to determine the robot's pose. In the future, this method could be updated to use multiple
   * tags.
   *
   * @param index the index of the VisionIO object to use
   * @return the robot pose based on vision data and distance to the AprilTag that is closest to the
   *     robot
   */
  private RobotPoseFromAprilTag getRobotPose(int index) {
    int targetCount = 0;
    Pose3d robotPoseFromClosestTarget = null;
    double closestTargetDistance = Double.MAX_VALUE;

    // "zero" the tag and robot poses such that old data is not used if no new data is available; in
    // terms of logging, we are assuming that a given VisionIO object won't see more than 2 tags at
    // once
    for (int i = 0; i < 2; i++) {
      Logger.recordOutput("Vision/TagPose" + index + "_" + i, new Pose2d());
      Logger.recordOutput("Vision/NVRobotPose" + index + "_" + i, new Pose2d());
    }

    for (PhotonTrackedTarget target : inputs[index].lastResult.getTargets()) {
      if (isValidTarget(target)) {
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Optional<Pose3d> tagPoseOptional = layout.getTagPose(target.getFiducialId());
        if (tagPoseOptional.isPresent()) {
          Pose3d tagPose = tagPoseOptional.get();
          Pose3d cameraPose = tagPose.transformBy(cameraToTarget.inverse());
          Pose3d robotPose = cameraPose.transformBy(camerasToRobots.get(index).inverse());

          Logger.recordOutput("Vision/TagPose" + index + "_" + targetCount, tagPose.toPose2d());
          Logger.recordOutput(
              "Vision/NVRobotPose" + index + "_" + targetCount, robotPose.toPose2d());

          double targetDistance =
              target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();

          // only consider tags that are within a certain distance of the robot
          if (targetDistance < VisionConstants.MAX_DISTANCE_TO_TARGET_METERS
              && targetDistance < closestTargetDistance) {
            closestTargetDistance = targetDistance;
            robotPoseFromClosestTarget = robotPose;
          }
        }
      }
      targetCount++;
    }

    return new RobotPoseFromAprilTag(robotPoseFromClosestTarget, closestTargetDistance);
  }

  /**
   * Returns the robot pose based on vision data and distance to the AprilTag that has the lowest
   * ambiguity.
   *
   * @param index the index of the VisionIO object to use
   * @return the robot pose based on vision data and distance to the AprilTag that has the lowest
   *     ambiguity
   */
  private RobotPoseFromAprilTag getRobotPoseOptimized(int index) {
    int targetCount = 0;
    Pose3d robotPoseFromClosestTarget = null;
    double lowestTargetAmbiguity = Double.MAX_VALUE;
    double targetDistance = Double.MAX_VALUE;

    // "zero" the tag and robot poses such that old data is not used if no new data is available; in
    // terms of logging, we are assuming that a given VisionIO object won't see more than 2 tags at
    // once
    for (int i = 0; i < 2; i++) {
      Logger.recordOutput("Vision/TagPose" + index + "_" + i, new Pose2d());
      Logger.recordOutput("Vision/NVRobotPose" + index + "_" + i, new Pose2d());
    }

    for (PhotonTrackedTarget target : inputs[index].lastResult.getTargets()) {
      if (isValidTarget(target)) {
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Optional<Pose3d> tagPoseOptional = layout.getTagPose(target.getFiducialId());
        if (tagPoseOptional.isPresent()) {
          Pose3d tagPose = tagPoseOptional.get();
          Pose3d cameraPose = tagPose.transformBy(cameraToTarget.inverse());
          Pose3d robotPose = cameraPose.transformBy(camerasToRobots.get(index).inverse());

          Logger.recordOutput("Vision/TagPose" + index + "_" + targetCount, tagPose.toPose2d());
          Logger.recordOutput(
              "Vision/NVRobotPose" + index + "_" + targetCount, robotPose.toPose2d());

          double targetAmbiguity = target.getPoseAmbiguity();

          // only consider tags that are within a certain distance of the robot
          if (targetAmbiguity < VisionConstants.MAXIMUM_AMBIGUITY
              && targetAmbiguity != -1
              && targetAmbiguity < lowestTargetAmbiguity) {
            lowestTargetAmbiguity = targetAmbiguity;
            robotPoseFromClosestTarget = robotPose;
            targetDistance =
                target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
          }
        }
      }
      targetCount++;
    }

    return new RobotPoseFromAprilTag(robotPoseFromClosestTarget, targetDistance);
  }

  /**
   * Checks if the given PhotonTrackedTarget is a valid target for robot pose calculation.
   *
   * @param target the PhotonTrackedTarget to check
   * @return true if the target is valid, false otherwise
   */
  private boolean isValidTarget(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1
        && target.getPoseAmbiguity() != -1
        && target.getPoseAmbiguity() < VisionConstants.MAXIMUM_AMBIGUITY
        && layout.getTagPose(target.getFiducialId()).isPresent();
  }
}
