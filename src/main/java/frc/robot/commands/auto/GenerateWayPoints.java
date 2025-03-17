package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;

public class GenerateWayPoints {

  private Pose2d startPose;
  private Pose2d endPose;
  private int resolution;

  public void GenerateWayPoints() {}

  public void configure(int resolution) {
    this.resolution = resolution;
  }

  public ArrayList<Translation2d> generate(Pose2d startPose, Pose2d endPose) {
    this.startPose = startPose;
    this.endPose = endPose;
    ArrayList<Translation2d> wayPoints = new ArrayList<Translation2d>();

    if (startPose.getRotation().getRadians() - endPose.getRotation().getRadians()
        == Units.degreesToRadians(1.0)) {
      for (int i = resolution + 1; i > 1; i--) {
        var midPoint =
            new Pose2d(
                (startPose.getX() + startPose.getX() + endPose.getX()) / resolution,
                (startPose.getY() + startPose.getY() + endPose.getY()) / resolution,
                startPose.getRotation());
        wayPoints.add(midPoint.getTranslation());
      }
    } else if (startPose.getRotation().getRadians() != endPose.getRotation().getRadians()) {
      for (int i = resolution + 1; i > 1; i--) {
        var midPoint =
            new Pose2d(
                (startPose.getX() + (startPose.getX() - endPose.getX())) / resolution,
                (startPose.getY() + (startPose.getY() - endPose.getY())) / resolution,
                new Rotation2d(
                    startPose.getRotation().getRadians()
                        + (startPose.getRotation().getRadians()
                                - endPose.getRotation().getRadians())
                            / resolution));
        wayPoints.add(midPoint.getTranslation());
      }
    }
    return wayPoints;
  }
}
