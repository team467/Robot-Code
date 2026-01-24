package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoAlignConstants {
  // Blue side Hub scoring positions
  public static final Pose2d BLUE_LEFT_BUMPER_POSE =
      new Pose2d(2.4235000610351562, 5.80819034576416, new Rotation2d(-0.9279060848928967));
  public static final Pose2d BLUE_RIGHT_BUMPER_POSE =
      new Pose2d(2.3498306274414062, 2.631226062774658, new Rotation2d(0.658769093080394));

  // Red side Hub scoring positions
  public static final Pose2d RED_LEFT_BUMPER_POSE =
      new Pose2d(14.214704513549805, 2.735670328140259, new Rotation2d(2.6182271695586263));
  public static final Pose2d RED_RIGHT_BUMPER_POSE =
      new Pose2d(13.94314956665039, 5.409444808959961, new Rotation2d(-2.4683594711511185));

  // Hub AprilTag IDs
  public static final int[] BLUE_HUB_TAG_IDS = {25, 26};
  public static final int[] RED_HUB_TAG_IDS = {9, 20};

  private AutoAlignConstants() {}
}
