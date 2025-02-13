package frc.robot.commands.drive;

import static frc.robot.FieldConstants.Reef.branchPositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.CoralStation;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.commands.auto.DriveToPose;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class FieldAlignment {

  @AutoLogOutput private Pose2d desiredCoralPose;
  @AutoLogOutput private int branchIndex;
  @AutoLogOutput private int closestReefFace;
  @AutoLogOutput private Pose2d closestReefFacePose;
  @AutoLogOutput private double[] reefFaceDistances = new double[6];

  private static final double CORAL_EFFECTOR_OFFSET = Units.inchesToMeters(12);
  private static final double BRANCH_TO_ROBOT_BACKUP = Units.inchesToMeters(18.375);
  private final Drive drive;

  public FieldAlignment(Drive drive) {
    this.drive = drive;
  }

  public Command alignToReef(boolean branchLeft) {
    return Commands.defer(
        () ->
            new DriveToPose(drive, getBranchPosition(branchLeft, closestReefFace())).withTimeout(5),
        Set.of(drive));
  }

  public Command faceReef(DoubleSupplier leftJoystickX, DoubleSupplier leftJoystickY) {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        leftJoystickX,
        leftJoystickY,
        () ->
            Rotation2d.fromDegrees(Reef.centerFaces[closestReefFace()].getRotation().getDegrees()));
  }

  public Command faceCoralStation(DoubleSupplier leftJoystickX, DoubleSupplier leftJoystickY) {
    return DriveCommands.joystickDriveAtAngle(
        drive, leftJoystickX, leftJoystickY, () -> getClosestCoralStationPosition().getRotation());
  }
  /**
   * Gets position of the branch we want.
   *
   * @param branchLeft
   * @return Command for getting branch postion.
   */
  public Pose2d getBranchPosition(boolean branchLeft, int closestReefFace) {
    int branch = closestReefFace * 2;
    if (branchLeft) {
      branch++;
    }
    branchIndex = branch;
    Pose2d branchPose = branchPositions.get(branch).get(ReefHeight.L1).toPose2d();
    ;
    desiredCoralPose =
        new Pose2d(
            branchPose.getX()
                - Units.inchesToMeters(BRANCH_TO_ROBOT_BACKUP)
                    * Math.cos(branchPose.getRotation().getRadians())
                - Units.inchesToMeters(CORAL_EFFECTOR_OFFSET)
                    * Math.sin(Units.degreesToRadians(branchPose.getRotation().getRadians())),
            branchPose.getY()
                - Units.inchesToMeters(BRANCH_TO_ROBOT_BACKUP)
                    * Math.sin(Units.degreesToRadians(branchPose.getRotation().getRadians()))
                + Units.inchesToMeters(BRANCH_TO_ROBOT_BACKUP)
                    * Math.cos(Units.degreesToRadians(branchPose.getRotation().getRadians())),
            branchPose.getRotation());
    return desiredCoralPose;
  }

  /**
   * Gets the closest coral station position, either leftCenterFace or rightCenterFace.
   *
   * @return Command for getting the closest coral station.
   */
  public Pose2d getClosestCoralStationPosition() {
    return closerToLeftCoralStation() ? CoralStation.leftCenterFace : CoralStation.rightCenterFace;
  }

  public boolean closerToLeftCoralStation() {
    double distanceToLeftStation =
        Math.hypot(
            Math.abs(drive.getPose().getX() - (CoralStation.leftCenterFace).getX()),
            Math.abs(drive.getPose().getY() - (CoralStation.leftCenterFace).getY()));
    double distanceToRightStation =
        Math.hypot(
            Math.abs(drive.getPose().getX() - (CoralStation.rightCenterFace).getX()),
            Math.abs(drive.getPose().getY() - (CoralStation.rightCenterFace).getY()));
    return (distanceToLeftStation < distanceToRightStation);
  }

  public int closestReefFace() {
    double[] reefFaceDistances = new double[6];

    for (int i = 0; i < 6; i++) {
      reefFaceDistances[i] =
          Math.hypot(
              Math.abs(drive.getPose().getX() - (Reef.centerFaces[i]).getX()),
              Math.abs(drive.getPose().getY() - (Reef.centerFaces[i]).getY()));
      this.reefFaceDistances[i] = reefFaceDistances[i];
    }
    int closestFace = 0;

    for (int i = 0; i < reefFaceDistances.length; i++) {
      if (reefFaceDistances[i] < reefFaceDistances[closestFace]) {
        closestFace = i;
      }
    }
    closestReefFace = closestFace;
    closestReefFacePose = Reef.centerFaces[closestFace];
    return closestFace;
  }
}
