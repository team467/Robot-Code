package frc.lib.io.vision;

public final class VisionConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private VisionConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  // the pose ambiguity must be less than this value for the target to be considered valid
  public static final double MAXIMUM_AMBIGUITY = 0.2;

  // the maximum distance between the robot and the target, for the target to be used to update the
  // robot's pose
  public static final double MAX_DISTANCE_TO_TARGET_METERS = 6.0;

  // the maximum distance between the robot's current estimated pose and the robot's pose derived
  // from the target to consider the two poses as having converged.
  public static final double POSE_DIFFERENCE_THRESHOLD_METERS = 0.5;
}
