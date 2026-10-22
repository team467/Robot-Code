package first.robot.util;

import static first.robot.constants.FieldConstants.fieldLength;
import static first.robot.constants.FieldConstants.fieldWidth;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

public final class AllianceFlipUtil {
  private AllianceFlipUtil() {}

  public static boolean shouldFlip() {
    return MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
  }

  public static double applyX(double x) {
    return shouldFlip() ? fieldLength - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? fieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Translation3d apply(Translation3d translation) {
    return new Translation3d(
        applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.PI) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static Pose2d reflectY(Pose2d pose) {
    return new Pose2d(pose.getX(), fieldWidth - pose.getY(), pose.getRotation().unaryMinus());
  }
}
