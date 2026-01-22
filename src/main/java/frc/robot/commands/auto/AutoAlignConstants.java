package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoAlignConstants {
  // Blue side Hub scoring positions
  public static final Pose2d BLUE_LEFT_BUMPER_POSE =
      new Pose2d(2.4235000610351562, 5.80819034576416, new Rotation2d(2.289625893394594));
  public static final Pose2d BLUE_RIGHT_BUMPER_POSE =
      new Pose2d(2.3498306274414062, 2.631226062774658, new Rotation2d(-2.607763411572618));

  // Red side Hub scoring positions
  public static final Pose2d RED_LEFT_BUMPER_POSE =
      new Pose2d(14.214704513549805, 2.735670328140259, new Rotation2d(-0.47951924886789843));
  public static final Pose2d RED_RIGHT_BUMPER_POSE =
      new Pose2d(13.94314956665039, 5.409444808959961, new Rotation2d(0.8106030376014262));

  // Hub AprilTag IDs
  public static final int[] BLUE_HUB_TAG_IDS = {25, 26};
  public static final int[] RED_HUB_TAG_IDS = {9, 20};

  private AutoAlignConstants() {}
}
