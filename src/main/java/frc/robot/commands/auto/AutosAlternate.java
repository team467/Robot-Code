package frc.robot.commands.auto;

import static frc.robot.FieldConstants.Reef.branchPositions;
import static frc.robot.commands.drive.FieldAlignment.BRANCH_TO_ROBOT_BACKUP;
import static frc.robot.commands.drive.FieldAlignment.CORAL_EFFECTOR_OFFSET;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.Orchestrator;
import frc.robot.commands.drive.FieldAlignment;
import frc.robot.subsystems.coral.CoralEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.fastalgae.FastAlgaeEffector;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutosAlternate {
  private final Drive drive;
  private final Orchestrator orchestrator;
  private final FieldAlignment fieldAlignment;
  private final CoralEffector coral;
  private final FastAlgaeEffector algae;
  private final Trigger hopperSeesCoral;
  private static final Supplier<Pose2d> A =
      () -> AllianceFlipUtil.apply(new Pose2d(new Translation2d(7.30, 6.14), new Rotation2d(3.14)));
  private static final Supplier<Pose2d> B =
      () -> AllianceFlipUtil.apply(new Pose2d(new Translation2d(7.30, 3.99), new Rotation2d(3.14)));
  private static final Supplier<Pose2d> C =
      () -> AllianceFlipUtil.apply(new Pose2d(new Translation2d(7.30, 1.89), new Rotation2d(3.14)));

  public AutosAlternate(
      Drive drive,
      Orchestrator orchestrator,
      FieldAlignment fieldAlignment,
      CoralEffector coral,
      Elevator elevator,
      FastAlgaeEffector algae) {
    this.drive = drive;
    this.orchestrator = orchestrator;
    this.fieldAlignment = fieldAlignment;
    this.coral = coral;
    this.algae = algae;
    hopperSeesCoral = new Trigger(coral::hopperSeesCoral).debounce(0.2);
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
    return Commands.runOnce(() -> drive.setPose(B.get()))
        .andThen(new StraightDriveToPose(-1, 0, 0, drive, 0.04));
  }

  public Command AScore(boolean left) {
    Supplier<Pose2d> scorePoint =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(5.682666778564453, 5.685015678405762),
                    new Rotation2d(0.8550528118433292)));
    return Commands.runOnce(() -> drive.setPose(A.get()))
        .andThen(Commands.waitSeconds(1))
        .andThen(
            new StraightDriveToPose(drive, scorePoint, 0.04)
                .withTimeout(4)
                .andThen(fieldAlignment.alignToReef(left).withTimeout(5))
                .andThen(orchestrator.moveElevatorToLevel(4).withTimeout(3))
                .andThen()
                // .andThen(orchestrator.placeCoral(4))
                .andThen(Commands.waitSeconds(1.2))
                .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)));
  }

  public Command sigmaATwoScore(boolean left) {
    Supplier<Pose2d> scorePoint1 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(5.17947006225, 5.5338854789), new Rotation2d(1.10714825504)));
    Logger.recordOutput("scorePoint1", AllianceFlipUtil.apply(scorePoint1.get()));
    Supplier<Pose2d> scorePoint2 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(new Translation2d(3.8055463, 6.839114), new Rotation2d(2.356194)));
    Logger.recordOutput("scorePoint2", AllianceFlipUtil.apply(scorePoint2.get()));
    Supplier<Pose2d> scorePoint3 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(new Translation2d(3.32467007637, 5.877366), new Rotation2d(2.22919)));
    Logger.recordOutput("scorePoint3", AllianceFlipUtil.apply(scorePoint3.get()));
    return Commands.runOnce(() -> drive.setPose(A.get()))
        .andThen(new StraightDriveToPose(drive, scorePoint1, 0.62))
        .withTimeout(1.5)
        .andThen(fieldAlignment.alignToReef(left))
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
        // .andThen(fieldAlignment.alignToCoralStation().withTimeout(2))
        // .andThen(orchestrator.intake().until(coral::hasCoral))
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

  public Command alphaAThreeScore(boolean left) {
    Supplier<Pose2d> scorePoint1 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(5.17947006225, 5.5338854789), new Rotation2d(1.10714825504)));
    Logger.recordOutput("scorePoint1", AllianceFlipUtil.apply(scorePoint1.get()));
    Supplier<Pose2d> scorePoint2 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(new Translation2d(3.8055463, 6.839114), new Rotation2d(2.356194)));
    Logger.recordOutput("scorePoint2", AllianceFlipUtil.apply(scorePoint2.get()));
    Supplier<Pose2d> scorePoint3 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(new Translation2d(3.32467007637, 5.877366), new Rotation2d(2.22919)));
    Logger.recordOutput("scorePoint3", AllianceFlipUtil.apply(scorePoint3.get()));
    return Commands.runOnce(() -> drive.setPose(A.get()))
        .andThen(new StraightDriveToPose(drive, scorePoint1, 0.62))
        .withTimeout(1.5)
        .andThen(fieldAlignment.alignToReef(left))
        .withTimeout(3)
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION))
        .andThen(new StraightDriveToPose(drive, scorePoint2, 0.9).withTimeout(0.8))
        // .andThen(
        // Commands.parallel(
        // fieldAlignment.alignToCoralStation().andThen(Commands.none()),
        // orchestrator.intake()).until(coral::hasCoral))
        .andThen(fieldAlignment.alignToCoralStation().withTimeout(2))
        .andThen(orchestrator.intake().until(coral::hasCoral))
        .andThen(
            Commands.race(
                new StraightDriveToPose(drive, scorePoint3, 1)
                    .withTimeout(1.5)
                    .andThen(fieldAlignment.alignToReef(true).withTimeout(2)),
                coral.stop()))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION))
        // .andThen(
        // Commands.parallel(
        // fieldAlignment.alignToCoralStation().andThen(Commands.none()),
        // orchestrator.intake()).until(coral::hasCoral))
        .andThen(fieldAlignment.alignToCoralStation().withTimeout(2))
        .andThen(orchestrator.intake().until(coral::hasCoral))
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

  public Command omegaAThreePointFiveScore(boolean left) {
    Supplier<Pose2d> scorePointA =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(new Translation2d(3.32467007637, 5.877366), new Rotation2d(2.22919)));
    Supplier<Pose2d> intakeInBetween =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(3.4687795639038086, 5.667267322540283),
                    new Rotation2d(2.077894778811894)));

    return new StraightDriveToPose(drive, scorePointA, 1)
        .withTimeout(2)
        .andThen(
            Commands.deadline(
                    Commands.parallel(
                            fieldAlignment.alignToReef(false).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(4).withTimeout(0.8).withTimeout(1.15))
                        .andThen(coral.dumpCoral().withTimeout(1)),
                    Commands.waitSeconds(0.9).andThen(orchestrator.removeAlgae(2)))
                .withTimeout(2.25))
        .andThen(Commands.parallel(algae.stop().withTimeout(0.01), coral.stop().withTimeout(0.01)))
        .andThen(
            Commands.deadline(
                Commands.race(
                    fieldAlignment.alignToCoralStation().andThen(Commands.waitSeconds(2.75)),
                    Commands.waitUntil(hopperSeesCoral).andThen(Commands.waitSeconds(0.2)),
                    Commands.waitUntil(coral::hasCoral)),
                Commands.parallel(
                    orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION),
                    Commands.waitSeconds(1).andThen(orchestrator.stowAlgae()))))
        .andThen(
            Commands.parallel(
                    new StraightDriveToPose(drive, intakeInBetween, 1),
                    orchestrator.intake().until(coral::hasCoral).andThen(coral.stop()))
                .withTimeout(1.5)
                .andThen(
                    Commands.parallel(
                            fieldAlignment.alignToReef(true).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(4).withTimeout(0.8),
                            coral.stop())
                        .withTimeout(1)))
        .andThen(coral.dumpCoral().withTimeout(1))
        .andThen(
            Commands.deadline(
                Commands.race(
                    fieldAlignment.alignToCoralStation().andThen(Commands.waitSeconds(2.75)),
                    Commands.waitUntil(hopperSeesCoral).andThen(Commands.waitSeconds(0.2)),
                    Commands.waitUntil(coral::hasCoral)),
                orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)))
        .andThen(
            Commands.parallel(
                    new StraightDriveToPose(drive, intakeInBetween, 1),
                    orchestrator.intake().until(coral::hasCoral).andThen(coral.stop()))
                .withTimeout(1.5)
                .andThen(
                    Commands.parallel(
                            fieldAlignment.alignToReef(true).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(3).withTimeout(0.75),
                            coral.stop())
                        .withTimeout(1)))
        .andThen(coral.dumpCoral().withTimeout(0.75))
        .andThen(
            Commands.deadline(
                Commands.race(
                    fieldAlignment.alignToCoralStation().andThen(Commands.waitSeconds(2.75)),
                    Commands.waitUntil(hopperSeesCoral).andThen(Commands.waitSeconds(0.2)),
                    Commands.waitUntil(coral::hasCoral)),
                orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)))
        .andThen(
            Commands.parallel(
                    new StraightDriveToPose(drive, intakeInBetween, 1),
                    orchestrator.intake().until(coral::hasCoral).andThen(coral.stop()))
                .withTimeout(1.5)
                .andThen(
                    Commands.parallel(
                            fieldAlignment.alignToReef(false).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(3).withTimeout(0.75),
                            coral.stop())
                        .withTimeout(1)))
        .andThen(coral.dumpCoral().withTimeout(1))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION));
  }

  public Command skibidiAFourScore(boolean left) {
    Supplier<Pose2d> scorePointA =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(new Translation2d(3.32467007637, 5.877366), new Rotation2d(2.22919)));
    Supplier<Pose2d> intakeInBetween =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(3.4687795639038086, 5.667267322540283),
                    new Rotation2d(2.077894778811894)));

    return new StraightDriveToPose(drive, scorePointA, 1)
        .withTimeout(2)
        .andThen(
            Commands.deadline(
                    Commands.parallel(
                            fieldAlignment.alignToReef(false).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(4).withTimeout(0.8).withTimeout(1.15))
                        .andThen(coral.dumpCoral().withTimeout(1)),
                    Commands.waitSeconds(0.9).andThen(orchestrator.removeAlgae(2)))
                .withTimeout(2.25))
        .andThen(Commands.parallel(algae.stop().withTimeout(0.01), coral.stop().withTimeout(0.01)))
        .andThen(
            Commands.deadline(
                Commands.race(
                    fieldAlignment.alignToCoralStation().andThen(Commands.waitSeconds(2.75)),
                    Commands.waitUntil(coral::hasCoral)),
                Commands.parallel(
                    orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION),
                    Commands.waitSeconds(1).andThen(orchestrator.stowAlgae()))))
        .andThen(
            Commands.parallel(
                    new StraightDriveToPose(drive, intakeInBetween, 1),
                    orchestrator.intake().until(coral::hasCoral).andThen(coral.stop()))
                .withTimeout(1.5)
                .andThen(
                    Commands.parallel(
                            fieldAlignment.alignToReef(true).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(4).withTimeout(0.8),
                            coral.stop())
                        .withTimeout(1)))
        .andThen(coral.dumpCoral().withTimeout(1))
        .andThen(
            Commands.deadline(
                Commands.race(
                    fieldAlignment.alignToCoralStation().andThen(Commands.waitSeconds(2.75)),
                    Commands.waitUntil(coral::hasCoral)),
                orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)))
        .andThen(
            Commands.parallel(
                    new StraightDriveToPose(drive, intakeInBetween, 1),
                    orchestrator.intake().until(coral::hasCoral).andThen(coral.stop()))
                .withTimeout(1.5)
                .andThen(
                    Commands.parallel(
                            fieldAlignment.alignToReef(true).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(3).withTimeout(0.75),
                            coral.stop())
                        .withTimeout(1)))
        .andThen(coral.dumpCoral().withTimeout(0.75))
        .andThen(
            Commands.deadline(
                Commands.race(
                    fieldAlignment.alignToCoralStation().andThen(Commands.waitSeconds(2.75)),
                    Commands.waitUntil(coral::hasCoral)),
                orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)))
        .andThen(
            Commands.parallel(
                    new StraightDriveToPose(drive, intakeInBetween, 1),
                    orchestrator.intake().until(coral::hasCoral).andThen(coral.stop()))
                .withTimeout(1.5)
                .andThen(
                    Commands.parallel(
                            fieldAlignment.alignToReef(false).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(3).withTimeout(0.75),
                            coral.stop())
                        .withTimeout(1)))
        .andThen(coral.dumpCoral().withTimeout(1))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION));
  }
  // use the command below to get the pathplanner implementation
  public Command C6Mpath2Coral() {
    return Commands.runOnce(() -> drive.setPose(C.get()))
        .andThen(drive.getAutonomousCommand("C6M Optimized").withTimeout(3))
        .andThen(
            Commands.parallel(
                fieldAlignment.alignToReef(true), orchestrator.moveElevatorToSetpoint(4)))
        .andThen(drive.getAutonomousCommand("6LI"))
        .andThen(
            Commands.parallel(
                    fieldAlignment.alignToCoralStation().andThen(Commands.none()),
                    orchestrator.intake().until(coral::hasCoral))
                .withTimeout(2.5))
        .andThen(drive.getAutonomousCommand("I6M"))
        .andThen(
            Commands.parallel(
                fieldAlignment.alignToReef(false), orchestrator.moveElevatorToSetpoint(4)));
  }

  public Command A2Mpath2Coral() {
    return Commands.runOnce(() -> drive.setPose(A.get()))
        .andThen(drive.getAutonomousCommand("A2M Optimized").withTimeout(3))
        .andThen(
            Commands.parallel(
                    fieldAlignment.alignToReef(true), orchestrator.moveElevatorToSetpoint(4))
                .withTimeout(3))
        .andThen(orchestrator.placeCoral(4))
        .andThen(
            Commands.parallel(
                    fieldAlignment.alignToCoralStation().andThen(Commands.none()),
                    orchestrator.intake().until(coral::hasCoral))
                .withTimeout(3))
        .andThen(drive.getAutonomousCommand("2LI"))
        .andThen(
            Commands.parallel(
                fieldAlignment.alignToReef(false), orchestrator.moveElevatorToSetpoint(4)))
        .andThen(orchestrator.placeCoral(4));
  }

  public Command BScore(boolean left) {
    return Commands.runOnce(() -> drive.setPose(B.get()))
        .andThen(Commands.waitSeconds(1))
        .andThen(
            fieldAlignment
                .alignToReef(left)
                .withTimeout(3)
                .andThen(orchestrator.moveElevatorToLevel(4).withTimeout(3))
                .andThen(coral.dumpCoral().withTimeout(4))
                // .andThen(orchestrator.placeCoral(4))
                .andThen(Commands.waitSeconds(1.2))
                .andThen(orchestrator.moveElevatorToSetpoint(ReefHeight.L1.height)));
  }

  public Command CScore(boolean left) {
    Supplier<Pose2d> scorePoint =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(5.579622268676758, 2.2639400959014893),
                    new Rotation2d(-0.982794168198375)));
    return Commands.runOnce(() -> drive.setPose(C.get()))
        .andThen(Commands.waitSeconds(1))
        .andThen(
            new StraightDriveToPose(drive, scorePoint, 0.04)
                .withTimeout(4)
                .andThen(fieldAlignment.alignToReef(left))
                .withTimeout(5)
                .andThen(orchestrator.placeCoral(4))
                .andThen(Commands.waitSeconds(1.2))
                .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)));
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
    return Commands.runOnce(() -> drive.setPose(C.get()))
        .andThen(
            Commands.parallel(
                orchestrator.moveElevatorBasedOnDistance(targetPose),
                new StraightDriveToPose(drive, targetPose)));
  }

  public Command sigmaCTwoScore(boolean left) {
    Supplier<Pose2d> scorePoint1 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(5.579622268676758, 2.2639400959014893),
                    new Rotation2d(-0.982794168198375)));
    Supplier<Pose2d> scorePoint2 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(1.6411257982254028, 1.703604817390442),
                    new Rotation2d(-2.2218729245897753)));
    Supplier<Pose2d> scorePoint3 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(2.899836778640747, 1.7935127019882202),
                    new Rotation2d(-2.1375256093137067)));
    return Commands.runOnce(() -> drive.setPose(C.get()))
        .andThen(new StraightDriveToPose(drive, scorePoint1, 0.62))
        .withTimeout(1.5)
        .andThen(fieldAlignment.alignToReef(left))
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
        // .andThen(fieldAlignment.alignToCoralStation().withTimeout(2))
        // .andThen(orchestrator.intake().until(coral::hasCoral))
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

  public Command alphaCThreeScore(boolean left) {
    Supplier<Pose2d> scorePoint1 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(5.579622268676758, 2.2639400959014893),
                    new Rotation2d(-0.982794168198375)));
    Supplier<Pose2d> scorePoint2 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(1.6411257982254028, 1.703604817390442),
                    new Rotation2d(-2.2218729245897753)));
    Supplier<Pose2d> scorePoint3 =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(2.899836778640747, 1.7935127019882202),
                    new Rotation2d(-2.1375256093137067)));
    return Commands.runOnce(() -> drive.setPose(C.get()))
        .andThen(new StraightDriveToPose(drive, scorePoint1, 0.62))
        .withTimeout(1.5)
        .andThen(fieldAlignment.alignToReef(left))
        .withTimeout(3)
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION))
        .andThen(new StraightDriveToPose(drive, scorePoint2, 0.9).withTimeout(0.8))
        // .andThen(
        // Commands.parallel(
        // fieldAlignment.alignToCoralStation().andThen(Commands.none()),
        // orchestrator.intake()).until(coral::hasCoral))
        .andThen(fieldAlignment.alignToCoralStation().withTimeout(2))
        .andThen(orchestrator.intake().until(coral::hasCoral))
        .andThen(
            Commands.race(
                new StraightDriveToPose(drive, scorePoint3, 1)
                    .withTimeout(1.5)
                    .andThen(fieldAlignment.alignToReef(true).withTimeout(2)),
                coral.stop()))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION))
        // .andThen(
        // Commands.parallel(
        // fieldAlignment.alignToCoralStation().andThen(Commands.none()),
        // orchestrator.intake()).until(coral::hasCoral))
        .andThen(fieldAlignment.alignToCoralStation().withTimeout(2))
        .andThen(orchestrator.intake().until(coral::hasCoral))
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

  public Command omegaCThreePointFiveScore(boolean left) {
    Supplier<Pose2d> scorePointC =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(3.382890462875366, 2.360543966293335),
                    new Rotation2d(-2.1375256093137067)));
    Supplier<Pose2d> intakeInBetween =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(3.3614182472229004, 2.4034883975982666),
                    new Rotation2d(-2.100386022965448)));
    return new StraightDriveToPose(drive, scorePointC, 1)
        .withTimeout(2)
        .andThen(
            Commands.deadline(
                    Commands.parallel(
                            fieldAlignment.alignToReef(false).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(4).withTimeout(0.8).withTimeout(1.15))
                        .andThen(coral.dumpCoral().withTimeout(1)),
                    Commands.waitSeconds(0.9).andThen(orchestrator.removeAlgae(2)))
                .withTimeout(2.25))
        .andThen(Commands.parallel(algae.stop().withTimeout(0.01), coral.stop().withTimeout(0.01)))
        .andThen(
            Commands.deadline(
                Commands.race(
                    fieldAlignment.alignToCoralStation().andThen(Commands.waitSeconds(2.75)),
                    Commands.waitUntil(hopperSeesCoral).andThen(Commands.waitSeconds(0.2)),
                    Commands.waitUntil(coral::hasCoral)),
                Commands.parallel(
                    orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION),
                    Commands.waitSeconds(1).andThen(orchestrator.stowAlgae()))))
        .andThen(
            Commands.parallel(
                    new StraightDriveToPose(drive, intakeInBetween, 1),
                    orchestrator.intake().until(coral::hasCoral).andThen(coral.stop()))
                .withTimeout(1.5)
                .andThen(
                    Commands.parallel(
                            fieldAlignment.alignToReef(true).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(4).withTimeout(0.8),
                            coral.stop())
                        .withTimeout(1)))
        .andThen(coral.dumpCoral().withTimeout(1))
        .andThen(
            Commands.deadline(
                Commands.race(
                    fieldAlignment.alignToCoralStation().andThen(Commands.waitSeconds(2.75)),
                    Commands.waitUntil(hopperSeesCoral).andThen(Commands.waitSeconds(0.2)),
                    Commands.waitUntil(coral::hasCoral)),
                orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)))
        .andThen(
            Commands.parallel(
                    new StraightDriveToPose(drive, intakeInBetween, 1),
                    orchestrator.intake().until(coral::hasCoral).andThen(coral.stop()))
                .withTimeout(1.5)
                .andThen(
                    Commands.parallel(
                            fieldAlignment.alignToReef(true).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(3).withTimeout(0.75),
                            coral.stop())
                        .withTimeout(1)))
        .andThen(coral.dumpCoral().withTimeout(0.75))
        .andThen(
            Commands.deadline(
                Commands.race(
                    fieldAlignment.alignToCoralStation().andThen(Commands.waitSeconds(2.75)),
                    Commands.waitUntil(hopperSeesCoral).andThen(Commands.waitSeconds(0.2)),
                    Commands.waitUntil(coral::hasCoral)),
                orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)))
        .andThen(
            Commands.parallel(
                    new StraightDriveToPose(drive, intakeInBetween, 1),
                    orchestrator.intake().until(coral::hasCoral).andThen(coral.stop()))
                .withTimeout(1.5)
                .andThen(
                    Commands.parallel(
                            fieldAlignment.alignToReef(false).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(3).withTimeout(0.75),
                            coral.stop())
                        .withTimeout(1)))
        .andThen(coral.dumpCoral().withTimeout(1))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION));
  }

  public Command skibidiCFourScore(boolean left) {
    Supplier<Pose2d> scorePointC =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(3.382890462875366, 2.360543966293335),
                    new Rotation2d(-2.1375256093137067)));
    Supplier<Pose2d> intakeInBetween =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(3.3614182472229004, 2.4034883975982666),
                    new Rotation2d(-2.100386022965448)));
    return new StraightDriveToPose(drive, scorePointC, 1)
        .withTimeout(2)
        .andThen(
            Commands.deadline(
                    Commands.parallel(
                            fieldAlignment.alignToReef(false).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(4).withTimeout(0.8).withTimeout(1.15))
                        .andThen(coral.dumpCoral().withTimeout(1)),
                    Commands.waitSeconds(0.9).andThen(orchestrator.removeAlgae(2)))
                .withTimeout(2.25))
        .andThen(Commands.parallel(algae.stop().withTimeout(0.01), coral.stop().withTimeout(0.01)))
        .andThen(
            Commands.deadline(
                Commands.race(
                    fieldAlignment.alignToCoralStation().andThen(Commands.waitSeconds(2.75)),
                    Commands.waitUntil(coral::hasCoral)),
                Commands.parallel(
                    orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION),
                    Commands.waitSeconds(1).andThen(orchestrator.stowAlgae()))))
        .andThen(
            Commands.parallel(
                    new StraightDriveToPose(drive, intakeInBetween, 1),
                    orchestrator.intake().until(coral::hasCoral).andThen(coral.stop()))
                .withTimeout(1.5)
                .andThen(
                    Commands.parallel(
                            fieldAlignment.alignToReef(true).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(4).withTimeout(0.8),
                            coral.stop())
                        .withTimeout(1)))
        .andThen(coral.dumpCoral().withTimeout(1))
        .andThen(
            Commands.deadline(
                Commands.race(
                    fieldAlignment.alignToCoralStation().andThen(Commands.waitSeconds(2.75)),
                    Commands.waitUntil(coral::hasCoral)),
                orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)))
        .andThen(
            Commands.parallel(
                    new StraightDriveToPose(drive, intakeInBetween, 1),
                    orchestrator.intake().until(coral::hasCoral).andThen(coral.stop()))
                .withTimeout(1.5)
                .andThen(
                    Commands.parallel(
                            fieldAlignment.alignToReef(true).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(3).withTimeout(0.75),
                            coral.stop())
                        .withTimeout(1)))
        .andThen(coral.dumpCoral().withTimeout(0.75))
        .andThen(
            Commands.deadline(
                Commands.race(
                    fieldAlignment.alignToCoralStation().andThen(Commands.waitSeconds(2.75)),
                    Commands.waitUntil(coral::hasCoral)),
                orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)))
        .andThen(
            Commands.parallel(
                    new StraightDriveToPose(drive, intakeInBetween, 1),
                    orchestrator.intake().until(coral::hasCoral).andThen(coral.stop()))
                .withTimeout(1.5)
                .andThen(
                    Commands.parallel(
                            fieldAlignment.alignToReef(false).withTimeout(1.5),
                            orchestrator.moveElevatorToLevel(3).withTimeout(0.75),
                            coral.stop())
                        .withTimeout(1)))
        .andThen(coral.dumpCoral().withTimeout(1))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION));
  }
}
