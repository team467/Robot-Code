package frc.lib.io.vision;

import static frc.lib.io.vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.RobotOdometry;
import frc.lib.utils.TunableNumber;
import frc.robot.FieldConstants;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * The Vision subsystem is responsible for updating the robot's estimated pose based on a collection
 * of cameras capturing AprilTags. The Vision subsystem is comprised of multiple VisionIO objects,
 * each of which is responsible for producing a single PhotonPipelineResult. There is a one-to-one
 * relationship between each VisionIO object and each co-processor (e.g., Raspberry Pi) running
 * PhotonVision.
 */
public class Vision extends SubsystemBase {
  private VisionIO[] visionIOs;
  private final VisionIOInputsAutoLogged[] ios;
  private double[] lastTimestamps;

  private AprilTagFieldLayout layout;

  private boolean isEnabled = true;
  private boolean isVisionUpdating = false;

  private RobotOdometry odometry;
  private final TunableNumber poseDifferenceThreshold =
      new TunableNumber("Vision/VisionPoseThreshold", POSE_DIFFERENCE_THRESHOLD_METERS);
  private final TunableNumber stdDevSlope = new TunableNumber("Vision/stdDevSlope", 0.50);
  private final TunableNumber stdDevPower = new TunableNumber("Vision/stdDevPower", 2.0);
  private final TunableNumber stdDevMultiTagFactor =
      new TunableNumber("Vision/stdDevMultiTagFactor", 0.2);

  /**
   * Create a new Vision subsystem. The number of VisionIO objects passed to the constructor must
   * match the number of robot-to-camera transforms returned by the RobotConfig singleton.
   *
   * @param visionIOs One or more VisionIO objects, each of which is responsible for producing a
   *     single single PhotonPipelineResult. There is a one-to-one relationship between each
   *     VisionIO object and each co-processor (e.g., Raspberry Pi) running PhotonVision.
   */
  public Vision(VisionIO[] visionIOs) {
    this.visionIOs = visionIOs;
    this.lastTimestamps = new double[visionIOs.length];
    this.ios = new VisionIOInputsAutoLogged[visionIOs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      this.ios[i] = new VisionIOInputsAutoLogged();
    }

    // retrieve a reference to the pose estimator singleton
    this.odometry = RobotOdometry.getInstance();

    // load and log all of the AprilTags in the field layout file
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
    isVisionUpdating = false;
    for (int i = 0; i < visionIOs.length; i++) {

      visionIOs[i].updateInputs(ios[i]);
      Logger.processInputs("Vision/" + i, ios[i]);

      // "zero" the robot poses and which tags are seen such that old data is not used if no new
      // data has been available in the past 0.1 seconds
      if (ios[i].lastCameraTimestamp + 0.1 < lastTimestamps[i]) {
        Logger.recordOutput("Vision/" + i + "/RobotPose", new Pose2d());
        for (AprilTag tag : this.layout.getTags()) {
          Logger.recordOutput("Vision/" + i + "/TagID" + "_" + tag.ID, false);
        }
      }

      // only process the vision data if the timestamp is newer than the last one
      if (lastTimestamps[i] < ios[i].lastCameraTimestamp) {
        lastTimestamps[i] = ios[i].lastCameraTimestamp;
        Pose2d estimatedRobotPose2d = ios[i].estimatedRobotPose.toPose2d();

        // only update the pose estimator if the vision subsystem is enabled
        if (isEnabled && !DriverStation.isAutonomous()) {
          // when updating the pose estimator, specify standard deviations based on the distance
          // from the robot to the AprilTag (the greater the distance, the less confident we are
          // in the measurement)
          odometry
              .getPoseEstimator()
              .addVisionMeasurement(
                  estimatedRobotPose2d,
                  ios[i].estimatedRobotPoseTimestamp,
                  getStandardDeviations(i, estimatedRobotPose2d));
          isVisionUpdating = true;
        }

        for (AprilTag tag : this.layout.getTags()) {
          Logger.recordOutput("Vision/" + i + "/TagID" + "_" + tag.ID, false);
        }
        for (int tagID : ios[i].estimatedRobotPoseTags) {
          Logger.recordOutput("Vision/" + i + "/TagID" + "_" + tagID, true);
        }

        Logger.recordOutput("Vision/" + i + "/RobotPose", estimatedRobotPose2d);
      }
    }
    Logger.recordOutput("Vision/IsEnabled", isEnabled);
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
   * Returns the estimated robot pose based on the most recent vision data. This method is used to
   * reset the robot's odometry based solely on the vision data.
   *
   * @return the estimated robot pose based on the most recent vision data
   */
  public Pose3d getBestRobotPose() {
    Pose3d robotPoseFromMostRecentData = null;
    double mostRecentTimestamp = 0.0;
    for (int i = 0; i < visionIOs.length; i++) {
      if (ios[i].estimatedRobotPoseTimestamp > mostRecentTimestamp) {
        robotPoseFromMostRecentData = ios[i].estimatedRobotPose;
        mostRecentTimestamp = ios[i].estimatedRobotPoseTimestamp;
      }
    }
    return robotPoseFromMostRecentData;
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
    for (int i = 0; i < visionIOs.length; i++) {
      Pose3d robotPose = ios[i].estimatedRobotPose;
      if (odometry
              .getPoseEstimator()
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
   * The standard deviations of the estimated pose, for use with a pose estimator. This should only
   * be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  private Matrix<N3, N1> getStandardDeviations(int index, Pose2d estimatedPose) {
    Matrix<N3, N1> estStdDevs = VecBuilder.fill(1, 1, 0.5);
    int[] tags = ios[index].estimatedRobotPoseTags;
    int numTags = 0;
    double avgDist = 0;
    for (int tag : tags) {
      Optional<Pose3d> tagPose = layout.getTagPose(tag);
      if (tagPose.isEmpty()) {
        continue;
      }
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) {
      return estStdDevs;
    }
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = estStdDevs.times(stdDevMultiTagFactor.get());
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > MAX_DISTANCE_TO_TARGET_METERS) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(stdDevSlope.get() * (Math.pow(avgDist, stdDevPower.get())));
    }

    return estStdDevs;
  }
}
