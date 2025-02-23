package frc.robot.commands.drive;

import static frc.robot.FieldConstants.Reef.branchPositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants.CoralStation;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.commands.auto.DriveToPose;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class FieldAlignment {
  @AutoLogOutput private int closestReefFace;
  @AutoLogOutput private Pose2d closestReefFacePose;
  @AutoLogOutput private double[] reefFaceDistances = new double[6];
  // How far left/right the robot needs to move to align with the coral effector instead of the
  // center of the robot
  private static final double CORAL_EFFECTOR_OFFSET = 4;
  // How far back the robot needs to move to align with the branch in a way that doesn't have the
  // robot impaling itself
  private static final double BRANCH_TO_ROBOT_BACKUP = -9.5;
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
        () -> AllianceFlipUtil.apply(Reef.centerFaces[closestReefFace()]).getRotation().rotateBy(Rotation2d.fromDegrees(180 )));
  }

  public Command faceCoralStation(DoubleSupplier leftJoystickX, DoubleSupplier leftJoystickY) {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        leftJoystickX,
        leftJoystickY,
        () -> getClosestCoralStationPosition().getRotation().rotateBy(Rotation2d.fromDegrees(180)));
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
    Pose2d branchPose =
        AllianceFlipUtil.apply(branchPositions.get(branch).get(ReefHeight.L1).toPose2d());
    return new Pose2d(
        branchPose.getX() // Move left robot relative
            - Units.inchesToMeters(BRANCH_TO_ROBOT_BACKUP)
                * Math.cos(branchPose.getRotation().getRadians())
            - Units.inchesToMeters(CORAL_EFFECTOR_OFFSET)
                * Math.sin(branchPose.getRotation().getRadians()),
        branchPose.getY() // Move back robot relative
            - Units.inchesToMeters(BRANCH_TO_ROBOT_BACKUP)
                * Math.sin(branchPose.getRotation().getRadians())
            + Units.inchesToMeters(CORAL_EFFECTOR_OFFSET)
                * Math.cos(branchPose.getRotation().getRadians()),
        branchPose.getRotation());
  }

  /**
   * Gets the closest coral station position, either leftCenterFace or rightCenterFace.
   *
   * @return the pose of the closest coral station.
   */
  public Pose2d getClosestCoralStationPosition() {
    return closerToLeftCoralStation()
        ? AllianceFlipUtil.apply(CoralStation.leftCenterFace)
        : AllianceFlipUtil.apply(CoralStation.rightCenterFace);
  }

  public boolean closerToLeftCoralStation() {
    double distanceToLeftStation =
        Math.hypot(
            Math.abs(
                drive.getPose().getX()
                    - (AllianceFlipUtil.apply(CoralStation.leftCenterFace)).getX()),
            Math.abs(
                drive.getPose().getY()
                    - (AllianceFlipUtil.apply(CoralStation.leftCenterFace)).getY()));
    double distanceToRightStation =
        Math.hypot(
            Math.abs(
                drive.getPose().getX()
                    - (AllianceFlipUtil.apply(CoralStation.rightCenterFace)).getX()),
            Math.abs(
                drive.getPose().getY()
                    - (AllianceFlipUtil.apply(CoralStation.rightCenterFace)).getY()));
    return (distanceToLeftStation < distanceToRightStation);
  }

  public int closestReefFace() {
    double[] reefFaceDistances = new double[6];

    for (int i = 0; i < 6; i++) {
      reefFaceDistances[i] =
          Math.hypot(
              Math.abs(
                  drive.getPose().getX() - (AllianceFlipUtil.apply(Reef.centerFaces[i])).getX()),
              Math.abs(
                  drive.getPose().getY() - (AllianceFlipUtil.apply(Reef.centerFaces[i])).getY()));
      this.reefFaceDistances[i] = reefFaceDistances[i];
    }
    int closestFace = 0;

    for (int i = 0; i < reefFaceDistances.length; i++) {
      if (reefFaceDistances[i] < reefFaceDistances[closestFace]) {
        closestFace = i;
      }
    }
    closestReefFace = closestFace;
    closestReefFacePose = AllianceFlipUtil.apply(Reef.centerFaces[closestFace]);
    return closestFace;
  }

  public void periodic() {
    closestReefFace = closestReefFace();
    closestReefFacePose = AllianceFlipUtil.apply(Reef.centerFaces[closestReefFace]);
  }
}
