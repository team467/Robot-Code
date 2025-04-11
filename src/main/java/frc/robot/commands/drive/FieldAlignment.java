package frc.robot.commands.drive;

import static frc.robot.FieldConstants.Reef.branchPositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.lib.utils.TunableNumber;
import frc.robot.FieldConstants.CoralStation;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.RobotState;
import frc.robot.commands.auto.StraightDriveToPose;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class FieldAlignment {
  @AutoLogOutput private int closestReefFace;
  @AutoLogOutput private Pose2d closestReefFacePose;
  @AutoLogOutput private double[] reefFaceDistances = new double[6];
  // How far left/right the robot needs to move to align with the coral effector instead of the
  // center of the robot
  public static final TunableNumber CORAL_EFFECTOR_OFFSET =
      new TunableNumber("FieldAlignment/CoralEffectorOffset", -12.5);
  // How far back the robot needs to move to align with the branch in a way that doesn't have the
  // robot impaling itself
  public static final TunableNumber BRANCH_TO_ROBOT_BACKUP =
      new TunableNumber("FieldAlignment/BranchToRobotBackup", -25.1);
  // you can change these values
  @AutoLogOutput public double CORAL_EFFECTOR_OFFSET_TUNING = -12.5;
  @AutoLogOutput private double BRANCH_TO_ROBOT_BACKUP_TUNING = -25.1;
  private final Drive drive;

  public FieldAlignment(Drive drive) {
    this.drive = drive;
  }

  public Command alignToReef(boolean branchLeft) {
    return Commands.defer(
        () ->
            new StraightDriveToPose(drive, getBranchPosition(branchLeft, closestReefFace()))
                .withTimeout(10),
        Set.of(drive));
  }

  public Command alignToCoralStation() {
    return Commands.defer(
        () -> new StraightDriveToPose(drive, getClosestCoralStationPositionForAlign()),
        Set.of(drive));
  }

  public Command alignToReefMatchTunable(boolean branchLeft) {
    return Commands.defer(
        () ->
            new StraightDriveToPose(
                    drive, getBranchPositionMatchTunable(branchLeft, closestReefFace()))
                .withTimeout(10),
        Set.of(drive));
  }

  public Command faceReef(DoubleSupplier leftJoystickX, DoubleSupplier leftJoystickY) {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        leftJoystickX,
        leftJoystickY,
        () -> AllianceFlipUtil.apply(Reef.centerFaces[closestReefFace()]).getRotation());
  }

  public Command faceCoralStation(DoubleSupplier leftJoystickX, DoubleSupplier leftJoystickY) {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        leftJoystickX,
        leftJoystickY,
        () ->
            getClosestCoralStationPosition()
                .get()
                .getRotation()
                .rotateBy(Rotation2d.fromDegrees(180)));
  }
  /**
   * Gets position of the branch we want.
   *
   * @param branchLeft
   * @return Command for getting branch postion.
   */
  public Supplier<Pose2d> getBranchPositionMatchTunable(boolean branchLeft, int closestReefFace) {
    int branch = closestReefFace * 2;
    if (branchLeft) {
      branch++;
    }
    Pose2d branchPose =
        AllianceFlipUtil.apply(branchPositions.get(branch).get(ReefHeight.L1).toPose2d());
    return () -> {
      Pose2d targetPose =
          new Pose2d(
              branchPose.getX() // Move left robot relative
                  - Units.inchesToMeters(BRANCH_TO_ROBOT_BACKUP_TUNING)
                      * Math.cos(branchPose.getRotation().getRadians())
                  - Units.inchesToMeters(CORAL_EFFECTOR_OFFSET_TUNING)
                      * Math.sin(branchPose.getRotation().getRadians()),
              branchPose.getY() // Move back robot relative
                  - Units.inchesToMeters(BRANCH_TO_ROBOT_BACKUP_TUNING)
                      * Math.sin(branchPose.getRotation().getRadians())
                  + Units.inchesToMeters(CORAL_EFFECTOR_OFFSET_TUNING)
                      * Math.cos(branchPose.getRotation().getRadians()),
              branchPose.getRotation());
      Logger.recordOutput("FieldAlignment/TargetPose", targetPose);
      return targetPose;
    };
  }

  public Supplier<Pose2d> getBranchPosition(boolean branchLeft, int closestReefFace) {
    int branch = closestReefFace * 2;
    if (branchLeft) {
      branch++;
    }
    Pose2d branchPose =
        AllianceFlipUtil.apply(branchPositions.get(branch).get(ReefHeight.L1).toPose2d());
    return () ->
        new Pose2d(
            branchPose.getX() // Move left robot relative
                - Units.inchesToMeters(BRANCH_TO_ROBOT_BACKUP.get())
                    * Math.cos(branchPose.getRotation().getRadians())
                - Units.inchesToMeters(CORAL_EFFECTOR_OFFSET.get())
                    * Math.sin(branchPose.getRotation().getRadians()),
            branchPose.getY() // Move back robot relative
                - Units.inchesToMeters(BRANCH_TO_ROBOT_BACKUP.get())
                    * Math.sin(branchPose.getRotation().getRadians())
                + Units.inchesToMeters(CORAL_EFFECTOR_OFFSET.get())
                    * Math.cos(branchPose.getRotation().getRadians()),
            branchPose.getRotation());
  }

  public Supplier<Pose2d> getBranchPosition(boolean branchLeft, IntSupplier closestReefFace) {
    return () -> {
      int branch = closestReefFace.getAsInt() * 2;
      if (branchLeft) {
        branch++;
      }
      Pose2d branchPose =
          AllianceFlipUtil.apply(branchPositions.get(branch).get(ReefHeight.L1).toPose2d());
      return new Pose2d(
          branchPose.getX() // Move left robot relative
              - Units.inchesToMeters(BRANCH_TO_ROBOT_BACKUP.get())
                  * Math.cos(branchPose.getRotation().getRadians())
              - Units.inchesToMeters(CORAL_EFFECTOR_OFFSET.get())
                  * Math.sin(branchPose.getRotation().getRadians()),
          branchPose.getY() // Move back robot relative
              - Units.inchesToMeters(BRANCH_TO_ROBOT_BACKUP.get())
                  * Math.sin(branchPose.getRotation().getRadians())
              + Units.inchesToMeters(CORAL_EFFECTOR_OFFSET.get())
                  * Math.cos(branchPose.getRotation().getRadians()),
          branchPose.getRotation());
    };
  }

  /**
   * Gets the closest coral station position, either leftCenterFace or rightCenterFace.
   *
   * @return the pose of the closest coral station.
   */
  public Supplier<Pose2d> getClosestCoralStationPositionForAlign() {
    return () -> {
      Pose2d targetPose = getClosestCoralStationPosition().get();
      return new Pose2d(
          targetPose.getX() // Move left robot relative
              - Units.inchesToMeters(-8) * Math.cos(targetPose.getRotation().getRadians()),
          targetPose.getY() // Move back robot relative
              - Units.inchesToMeters(-8) * Math.sin(targetPose.getRotation().getRadians()),
          targetPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    };
  }

  public Supplier<Pose2d> getClosestCoralStationPosition() {
    return () ->
        closerToLeftCoralStation()
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
    RobotState.getInstance().ClosestReefFace = closestReefFace;
    closestReefFacePose = AllianceFlipUtil.apply(Reef.centerFaces[closestReefFace]);
  }

  public Command updateMidMatchTunableOffsets(Supplier<Integer> pov) {
    return Commands.run(
        () -> {
          switch (pov.get()) {
            case 0 -> CORAL_EFFECTOR_OFFSET_TUNING += -0.1;
            case 90 -> BRANCH_TO_ROBOT_BACKUP_TUNING += 0.1;
            case 180 -> CORAL_EFFECTOR_OFFSET_TUNING += 0.1;
            case 270 -> BRANCH_TO_ROBOT_BACKUP_TUNING += -0.1;
          }
        });
  }
}
