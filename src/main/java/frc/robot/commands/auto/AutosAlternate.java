package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.lib.utils.ChoreoVariables;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.Orchestrator;
import frc.robot.commands.drive.FieldAlignment;
import frc.robot.subsystems.coral.CoralEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import static frc.robot.FieldConstants.Reef.branchPositions;
import static frc.robot.commands.drive.FieldAlignment.BRANCH_TO_ROBOT_BACKUP;
import static frc.robot.commands.drive.FieldAlignment.CORAL_EFFECTOR_OFFSET;

public class AutosAlternate {

  private final Drive drive;
  private final Orchestrator orchestrator;
  private final FieldAlignment fieldAlignment;
  private final CoralEffector coral;
  private final Elevator elevator;

  public AutosAlternate(
      Drive drive,
      Orchestrator orchestrator,
      FieldAlignment fieldAlignment,
      CoralEffector coral,
      Elevator elevator) {
    this.drive = drive;
    this.orchestrator = orchestrator;
    this.fieldAlignment = fieldAlignment;
    this.coral = coral;
    this.elevator = elevator;
  }

  public Command BScoreHopeAndPray() {
    return Commands.defer(
            () -> {
              drive.runVelocity(new ChassisSpeeds(-Units.inchesToMeters(30), 0, 0));
              return null;
            },
            Set.of(drive))
        .andThen(Commands.waitSeconds(5))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(2))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION));
  }

  public Command zeroPiece() {
    Supplier<Pose2d> B = () -> AllianceFlipUtil.apply(ChoreoVariables.getPose("B"));
    return Commands.runOnce(() -> drive.setPose(B.get()))
        .andThen(new StraightDriveToPose(-1, 0, 0, drive, 0.04));
  }

  public Command elevatorRelativeToPose(boolean branchLeft, int closestReefFace) {
    double targetPosition = ReefHeight.L4.height;
    int branch = closestReefFace * 2;
    Pose2d branchPose =
        AllianceFlipUtil.apply(branchPositions.get(branch).get(ReefHeight.L1).toPose2d());
    Supplier<Pose2d> targetPose =
        () ->
            AllianceFlipUtil.apply(
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
                    branchPose.getRotation()));
    Supplier<Pose2d> C =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    ChoreoVariables.getPose("C").getTranslation(),
                    ChoreoVariables.getPose("C").getRotation().plus(Rotation2d.k180deg)));
    Logger.recordOutput("C", AllianceFlipUtil.apply(C.get()));
    return elevator
        .setHoldPosition(elevator.getPosition())
        .andThen(Commands.runOnce(() -> drive.setPose(C.get())))
        .andThen(
            Commands.parallel(
                orchestrator.moveElevatorBasedOnDistance(targetPose),
                new StraightDriveToPose(drive, targetPose)));
  }

  public Command AScore(boolean left) {
    Supplier<Pose2d> A =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    ChoreoVariables.getPose("A").getTranslation(),
                    ChoreoVariables.getPose("A").getRotation().plus(Rotation2d.k180deg)));
    Supplier<Pose2d> scorePoint =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(5.682666778564453, 5.685015678405762),
                    new Rotation2d(0.8550528118433292)));
    return Commands.runOnce(() -> drive.setPose(A.get()))
        .andThen(Commands.waitSeconds(1))
        .andThen(
            new StraightDriveToPose(drive, scorePoint, 0.62)
                .withTimeout(4)
                .andThen(fieldAlignment.alignToReef(left))
                .withTimeout(5)
                .andThen(orchestrator.placeCoral(4))
                .andThen(Commands.waitSeconds(1.2))
                .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)));
  }

  public Command sigmaATwoScore() {
    Supplier<Pose2d> A =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    ChoreoVariables.getPose("A").getTranslation(),
                    ChoreoVariables.getPose("A").getRotation().plus(Rotation2d.k180deg)));
    Supplier<Pose2d> scorePoint1 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(5.17947006225, 5.5338854789), new Rotation2d(1.10714825504)));
    Logger.recordOutput("scorePoint1A", AllianceFlipUtil.apply(scorePoint1.get()));
    Supplier<Pose2d> scorePoint2 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(new Translation2d(3.8055463, 6.839114), new Rotation2d(2.356194)));
    Logger.recordOutput("scorePoint2A", AllianceFlipUtil.apply(scorePoint2.get()));
    Supplier<Pose2d> scorePoint3 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(new Translation2d(3.32467007637, 5.877366), new Rotation2d(2.22919)));
    Logger.recordOutput("scorePoint3A", AllianceFlipUtil.apply(scorePoint3.get()));
    return elevator
        .setHoldPosition(elevator.getPosition())
        .andThen(Commands.runOnce(() -> drive.setPose(A.get())))
        .andThen(new StraightDriveToPose(drive, scorePoint1, 0.62))
        .withTimeout(1.5)
        .andThen(fieldAlignment.alignToReef(false))
        .withTimeout(3)
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(
            orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION).withTimeout(1))
        .andThen(new StraightDriveToPose(drive, scorePoint2, 0.9).withTimeout(0.8))
        .andThen(
            Commands.parallel(
                    fieldAlignment.alignToCoralStation().andThen(Commands.none()),
                    orchestrator.intake())
                .until(coral::hasCoral))
        .andThen(
            Commands.race(
                new StraightDriveToPose(drive, scorePoint3, 1)
                    .withTimeout(1.5)
                    .andThen(fieldAlignment.alignToReef(true).withTimeout(2)),
                coral.stop()))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION));
  }

  public Command alphaAThreeScore() {
    Supplier<Pose2d> A =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    ChoreoVariables.getPose("A").getTranslation(),
                    ChoreoVariables.getPose("A").getRotation().plus(Rotation2d.k180deg)));
    Supplier<Pose2d> scorePoint1 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(5.17947006225, 5.5338854789), new Rotation2d(1.10714825504)));
    Logger.recordOutput("scorePoint1A", AllianceFlipUtil.apply(scorePoint1.get()));
    Supplier<Pose2d> scorePoint2 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(new Translation2d(3.8055463, 6.839114), new Rotation2d(2.356194)));
    Logger.recordOutput("scorePoint2A", AllianceFlipUtil.apply(scorePoint2.get()));
    Supplier<Pose2d> scorePoint3 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(new Translation2d(3.32467007637, 5.877366), new Rotation2d(2.22919)));
    Logger.recordOutput("scorePoint3A", AllianceFlipUtil.apply(scorePoint3.get()));
    return elevator
        .setHoldPosition(elevator.getPosition())
        .andThen(Commands.runOnce(() -> drive.setPose(A.get())))
        .andThen(new StraightDriveToPose(drive, scorePoint1, 0.62))
        .withTimeout(1.5)
        .andThen(
            Commands.parallel(
                fieldAlignment.alignToReef(false).withTimeout(3),
                orchestrator.moveElevatorToLevel(false, 3)))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(
            orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION).withTimeout(1))
        .andThen(new StraightDriveToPose(drive, scorePoint2, 0.9).withTimeout(0.8))
        .andThen(
            Commands.parallel(
                    fieldAlignment.alignToCoralStation().andThen(Commands.none()),
                    orchestrator.intake())
                .until(coral::hasCoral))
        .andThen(
            Commands.race(
                new StraightDriveToPose(drive, scorePoint3, 1)
                    .withTimeout(1.5)
                    .andThen(
                        Commands.parallel(
                            fieldAlignment.alignToReef(true).withTimeout(2),
                            orchestrator.moveElevatorToLevel(false, 3))),
                coral.stop()))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION))
        .andThen(
            Commands.parallel(
                    fieldAlignment.alignToCoralStation().andThen(Commands.none()),
                    orchestrator.intake())
                .until(coral::hasCoral))
        .andThen(
            Commands.race(
                new StraightDriveToPose(drive, scorePoint3, 1)
                    .withTimeout(1.5)
                    .andThen(
                        Commands.parallel(
                            fieldAlignment.alignToReef(false).withTimeout(2),
                            orchestrator.moveElevatorToLevel(false, 3))),
                coral.stop()))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION));
  }

  public Command skibidiAFourScore() {
    Supplier<Pose2d> A =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    ChoreoVariables.getPose("A").getTranslation(),
                    ChoreoVariables.getPose("A").getRotation().plus(Rotation2d.k180deg)));
    Supplier<Pose2d> scorePoint1 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(5.17947006225, 5.5338854789), new Rotation2d(1.10714825504)));
    Logger.recordOutput("scorePoint1A", AllianceFlipUtil.apply(scorePoint1.get()));
    Supplier<Pose2d> scorePoint2 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(new Translation2d(3.8055463, 6.839114), new Rotation2d(2.356194)));
    Logger.recordOutput("scorePoint2A", AllianceFlipUtil.apply(scorePoint2.get()));
    Supplier<Pose2d> scorePoint3 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(new Translation2d(3.32467007637, 5.877366), new Rotation2d(2.22919)));
    Logger.recordOutput("scorePoint3A", AllianceFlipUtil.apply(scorePoint3.get()));
    return elevator
        .setHoldPosition(elevator.getPosition())
        .andThen(Commands.runOnce(() -> drive.setPose(A.get())))
        .andThen(new StraightDriveToPose(drive, scorePoint1, 0.62))
        .withTimeout(1.5)
        .andThen(
            Commands.parallel(
                fieldAlignment.alignToReef(false).withTimeout(3),
                orchestrator.moveElevatorToLevel(false, 3)))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(
            orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION).withTimeout(1))
        .andThen(new StraightDriveToPose(drive, scorePoint2, 0.9).withTimeout(0.8))
        .andThen(
            Commands.parallel(
                    fieldAlignment.alignToCoralStation().andThen(Commands.none()),
                    orchestrator.intake())
                .withTimeout(1))
        .andThen(
            Commands.race(
                new StraightDriveToPose(drive, scorePoint3, 1)
                    .withTimeout(1.5)
                    .andThen(
                        Commands.parallel(
                            fieldAlignment.alignToReef(false).withTimeout(2),
                            orchestrator.moveElevatorToLevel(false, 3))),
                coral.stop()))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION))
        .andThen(
            Commands.parallel(
                    fieldAlignment.alignToCoralStation().andThen(Commands.none()),
                    orchestrator.intake())
                .withTimeout(1))
        .andThen(
            Commands.race(
                new StraightDriveToPose(drive, scorePoint3, 1)
                    .withTimeout(1.5)
                    .andThen(
                        Commands.parallel(
                            fieldAlignment.alignToReef(true).withTimeout(2),
                            orchestrator.moveElevatorToLevel(false, 3))),
                coral.stop()))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION))
        .andThen(
            Commands.parallel(
                    fieldAlignment.alignToCoralStation().andThen(Commands.none()),
                    orchestrator.intake())
                .withTimeout(1))
        .andThen(
            Commands.race(
                new StraightDriveToPose(drive, scorePoint1, 1)
                    .withTimeout(3)
                    .andThen(
                        Commands.parallel(
                            fieldAlignment.alignToReef(true).withTimeout(2),
                            orchestrator.moveElevatorToLevel(false, 3))),
                coral.stop()))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION));
  }

  public Command BScore(boolean left) {
    Supplier<Pose2d> B =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    ChoreoVariables.getPose("B").getTranslation(),
                    ChoreoVariables.getPose("B").getRotation().plus(Rotation2d.k180deg)));
    return Commands.runOnce(() -> drive.setPose(B.get()))
        .andThen(Commands.waitSeconds(1))
        .andThen(
            fieldAlignment
                .alignToReef(left)
                .withTimeout(3)
                .andThen(orchestrator.placeCoral(4))
                .andThen(Commands.waitSeconds(1.2))
                .andThen(orchestrator.moveElevatorToSetpoint(ReefHeight.L1.height)));
  }

  public Command CScore(boolean left) {
    Supplier<Pose2d> C =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    ChoreoVariables.getPose("C").getTranslation(),
                    ChoreoVariables.getPose("C").getRotation().plus(Rotation2d.k180deg)));
    Supplier<Pose2d> scorePoint =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(5.579622268676758, 2.2639400959014893),
                    new Rotation2d(-0.982794168198375)));
    return Commands.runOnce(() -> drive.setPose(C.get()))
        .andThen(Commands.waitSeconds(1))
        .andThen(
            new StraightDriveToPose(drive, scorePoint, 0.62)
                .withTimeout(4)
                .andThen(fieldAlignment.alignToReef(left))
                .withTimeout(5)
                .andThen(orchestrator.placeCoral(4))
                .andThen(Commands.waitSeconds(1.2))
                .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)));
  }

  public Command sigmaCTwoScore() {
    Supplier<Pose2d> C =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    ChoreoVariables.getPose("C").getTranslation(),
                    ChoreoVariables.getPose("C").getRotation().plus(Rotation2d.k180deg)));
    Supplier<Pose2d> scorePoint1 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(5.579622268676758, 2.2639400959014893),
                    new Rotation2d(-0.982794168198375)));
    Logger.recordOutput("scorePoint1C", AllianceFlipUtil.apply(scorePoint1.get()));
    Supplier<Pose2d> scorePoint2 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(1.6411257982254028, 1.703604817390442),
                    new Rotation2d(-2.2218729245897753)));
    Logger.recordOutput("scorePoint2C", AllianceFlipUtil.apply(scorePoint2.get()));
    Supplier<Pose2d> scorePoint3 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(2.899836778640747, 1.7935127019882202),
                    new Rotation2d(-2.1375256093137067)));
    Logger.recordOutput("scorePoint3C", AllianceFlipUtil.apply(scorePoint3.get()));
    return elevator
        .setHoldPosition(elevator.getPosition())
        .andThen(Commands.runOnce(() -> drive.setPose(C.get())))
        .andThen(new StraightDriveToPose(drive, scorePoint1, 0.62))
        .withTimeout(1.5)
        .andThen(fieldAlignment.alignToReef(true))
        .withTimeout(3)
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(
            orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION).withTimeout(1))
        .andThen(new StraightDriveToPose(drive, scorePoint2, 0.9).withTimeout(0.8))
        .andThen(
            Commands.parallel(
                    fieldAlignment.alignToCoralStation().andThen(Commands.none()),
                    orchestrator.intake())
                .until(coral::hasCoral))
        .andThen(
            Commands.race(
                new StraightDriveToPose(drive, scorePoint3, 1)
                    .withTimeout(1.5)
                    .andThen(fieldAlignment.alignToReef(true).withTimeout(2)),
                coral.stop()))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION));
  }

  public Command alphaCThreeScore() {
    Supplier<Pose2d> C =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    ChoreoVariables.getPose("C").getTranslation(),
                    ChoreoVariables.getPose("C").getRotation().plus(Rotation2d.k180deg)));
    Supplier<Pose2d> scorePoint1 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(5.579622268676758, 2.2639400959014893),
                    new Rotation2d(-0.982794168198375)));
    Logger.recordOutput("scorePoint1C", AllianceFlipUtil.apply(scorePoint1.get()));
    Supplier<Pose2d> scorePoint2 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(1.6411257982254028, 1.703604817390442),
                    new Rotation2d(-2.2218729245897753)));
    Logger.recordOutput("scorePoint2C", AllianceFlipUtil.apply(scorePoint2.get()));
    Supplier<Pose2d> scorePoint3 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(2.899836778640747, 1.7935127019882202),
                    new Rotation2d(-2.1375256093137067)));
    Logger.recordOutput("scorePoint3C", AllianceFlipUtil.apply(scorePoint3.get()));
    return elevator
        .setHoldPosition(elevator.getPosition())
        .andThen(Commands.runOnce(() -> drive.setPose(C.get())))
        .andThen(new StraightDriveToPose(drive, scorePoint1, 0.62))
        .withTimeout(1.5)
        .andThen(
            Commands.parallel(
                fieldAlignment.alignToReef(true).withTimeout(3),
                orchestrator.moveElevatorToLevel(false, 3)))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(
            orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION).withTimeout(1))
        .andThen(new StraightDriveToPose(drive, scorePoint2, 0.9).withTimeout(0.8))
        .andThen(
            Commands.parallel(
                    fieldAlignment.alignToCoralStation().andThen(Commands.none()),
                    orchestrator.intake())
                .until(coral::hasCoral))
        .andThen(
            Commands.race(
                new StraightDriveToPose(drive, scorePoint3, 1)
                    .withTimeout(1.5)
                    .andThen(
                        Commands.parallel(
                            fieldAlignment.alignToReef(true).withTimeout(2),
                            orchestrator.moveElevatorToLevel(false, 3))),
                coral.stop()))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION))
        .andThen(
            Commands.parallel(
                    fieldAlignment.alignToCoralStation().andThen(Commands.none()),
                    orchestrator.intake())
                .until(coral::hasCoral))
        .andThen(
            Commands.race(
                new StraightDriveToPose(drive, scorePoint3, 1)
                    .withTimeout(1.5)
                    .andThen(
                        Commands.parallel(
                            fieldAlignment.alignToReef(false).withTimeout(2),
                            orchestrator.moveElevatorToLevel(false, 3))),
                coral.stop()))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION));
  }

  public Command skibidiCFourScore() {
    Supplier<Pose2d> C =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    ChoreoVariables.getPose("C").getTranslation(),
                    ChoreoVariables.getPose("C").getRotation().plus(Rotation2d.k180deg)));
    Supplier<Pose2d> scorePoint1 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(5.579622268676758, 2.2639400959014893),
                    new Rotation2d(-0.982794168198375)));
    Logger.recordOutput("scorePoint1C", AllianceFlipUtil.apply(scorePoint1.get()));
    Supplier<Pose2d> scorePoint2 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(1.6411257982254028, 1.703604817390442),
                    new Rotation2d(-2.2218729245897753)));
    Logger.recordOutput("scorePoint2C", AllianceFlipUtil.apply(scorePoint2.get()));
    Supplier<Pose2d> scorePoint3 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(2.899836778640747, 1.7935127019882202),
                    new Rotation2d(-2.1375256093137067)));
    Logger.recordOutput("scorePoint3C", AllianceFlipUtil.apply(scorePoint3.get()));
    return elevator
        .setHoldPosition(elevator.getPosition())
        .andThen(Commands.runOnce(() -> drive.setPose(C.get())))
        .andThen(new StraightDriveToPose(drive, scorePoint1, 0.62))
        .withTimeout(1.5)
        .andThen(
            Commands.parallel(
                fieldAlignment.alignToReef(true).withTimeout(3),
                orchestrator.moveElevatorToLevel(false, 3)))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(
            orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION).withTimeout(1))
        .andThen(new StraightDriveToPose(drive, scorePoint2, 0.9).withTimeout(0.8))
        .andThen(
            Commands.parallel(
                    fieldAlignment.alignToCoralStation().andThen(Commands.none()),
                    orchestrator.intake())
                .withTimeout(1))
        .andThen(
            Commands.race(
                new StraightDriveToPose(drive, scorePoint3, 1)
                    .withTimeout(1.5)
                    .andThen(
                        Commands.parallel(
                            fieldAlignment.alignToReef(true).withTimeout(2),
                            orchestrator.moveElevatorToLevel(false, 3))),
                coral.stop()))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION))
        .andThen(
            Commands.parallel(
                    fieldAlignment.alignToCoralStation().andThen(Commands.none()),
                    orchestrator.intake())
                .withTimeout(1))
        .andThen(
            Commands.race(
                new StraightDriveToPose(drive, scorePoint3, 1)
                    .withTimeout(1.5)
                    .andThen(
                        Commands.parallel(
                            fieldAlignment.alignToReef(false).withTimeout(2),
                            orchestrator.moveElevatorToLevel(false, 3))),
                coral.stop()))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION))
        .andThen(
            Commands.parallel(
                    fieldAlignment.alignToCoralStation().andThen(Commands.none()),
                    orchestrator.intake())
                .withTimeout(1))
        .andThen(
            Commands.race(
                new StraightDriveToPose(drive, scorePoint1, 1)
                    .withTimeout(3)
                    .andThen(
                        Commands.parallel(
                            fieldAlignment.alignToReef(false).withTimeout(2),
                            orchestrator.moveElevatorToLevel(false, 3))),
                coral.stop()))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION));
  }
}
