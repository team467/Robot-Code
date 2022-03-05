package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum StartingConfiguration {
  RED_TWO_CC("", new Pose2d());


  public enum Color {
    RED,
    BLUE
  }

  public enum BallCount {
    ONE,
    TWO
  }

  public enum StartingPosition {
    LEFT,
    MIDDLE,
    RIGHT
  }

  private final String name;
  private final Pose2d defaultPose;
    StartingConfiguration(String name, Pose2d defaultPose) {
    this.name = name;
    this.defaultPose = defaultPose;
  }

  public String getName() {
    return name;
  }

  public Pose2d getDefaultPose() {
    return defaultPose;
  }
}
