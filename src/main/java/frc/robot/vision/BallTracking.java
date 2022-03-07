package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class BallTracking {
  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision").getSubTable("BallTracking");
  private static final NetworkTable redTable = table.getSubTable("Red");
  private static final NetworkTable blueTable = table.getSubTable("Blue");
  /**
   * @return If the camera sees a red ball
   */
  public static boolean hasRedBall() {
    return redTable.getEntry("IsValid").getBoolean(false);
  }

  /**
   * @return If the camera sees a blue ball
   */
  public static boolean hasBlueBall() {
    return blueTable.getEntry("IsValid").getBoolean(false);
  }

  /**
   * @return If the camera sees a ball of the current alliance
   */
  public static boolean hasBall() {
    return blueTable.getEntry("IsValid").getBoolean(false);
  }

  /**
   * @return Distance to the red ball in meters
   */
  public static double getRedDistance() {
    return redTable.getEntry("Distance").getDouble(0);
  }

  /**
   * @return Distance to the blue ball in meters
   */
  public static double getBlueDistance() {
    return blueTable.getEntry("Distance").getDouble(0);
  }


  /**
   * @return Distance to the ball of the current alliance in meters
   */
  public static double getDistance() {
    switch (DriverStation.getAlliance()) {
      case Red:
        return getRedDistance();
      default:
      case Blue:
        return getBlueDistance();
    }
  }

  /**
   * @return Angle to the red ball off vertical in degrees
   */
  public static double getRedAngle() {
    return redTable.getEntry("angle").getDouble(0);
  }

  /**
   * @return Angle to the blue ball off vertical in degrees
   */
  public static double getBlueAngle() {
    return blueTable.getEntry("angle").getDouble(0);
  }

  /**
   * @return Angle to the ball of the current alliance off vertical in degrees
   */
  public static double getAngle() {
    switch (DriverStation.getAlliance()) {
      case Red:
        return getRedAngle();
      default:
      case Blue:
        return getBlueAngle();
    }
  }

  /**
   * @return Rotation2d to the red ball
   */
  public static Rotation2d getRedRotation2d() {
    return Rotation2d.fromDegrees(getRedAngle());
  }

  /**
   * @return Rotation2d to the blue ball
   */
  public static Rotation2d getBlueRotation2d() {
    return Rotation2d.fromDegrees(getBlueAngle());
  }

  /**
   * @return Rotation2d to the ball of the current alliance
   */
  public static Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getAngle());
  }

  /**
   * @return Translation2d to the red ball
   */
  public static Translation2d getRedTranslation2d() {
    return new Translation2d(getRedDistance(), getRedRotation2d());
  }

  /**
   * @return Translation2d to the blue ball
   */
  public static Translation2d getBlueTranslation2d() {
    return new Translation2d(getBlueDistance(), getBlueRotation2d());
  }

    /**
     * @return Translation2d to the ball of the current alliance
     */
    public static Translation2d getTranslation2d() {
      return new Translation2d(getDistance(), getRotation2d());
  }
}
