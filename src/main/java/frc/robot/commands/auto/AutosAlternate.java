package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.subsystems.elevator.ElevatorConstants;
import java.util.Set;
import java.util.function.Supplier;

public class AutosAlternate {
  private final Drive drive;
  private final Orchestrator orchestrator;
  private final FieldAlignment fieldAlignment;
  private final CoralEffector coral;

  public AutosAlternate(
      Drive drive, Orchestrator orchestrator, FieldAlignment fieldAlignment, CoralEffector coral) {
    this.drive = drive;
    this.orchestrator = orchestrator;
    this.fieldAlignment = fieldAlignment;
    this.coral = coral;
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

  public Command B1L() {
    Supplier<Pose2d> B = () -> AllianceFlipUtil.apply(ChoreoVariables.getPose("B"));
    Supplier<Pose2d> L1 = () -> AllianceFlipUtil.apply(ChoreoVariables.getPose("L1"));
    return Commands.runOnce(() -> drive.setPose(B.get()))
        .andThen(new StraightDriveToPose(drive, L1, 0.04));
  }

  public Command B1R() {
    Supplier<Pose2d> B = () -> AllianceFlipUtil.apply(ChoreoVariables.getPose("B"));
    Supplier<Pose2d> R1 = () -> AllianceFlipUtil.apply(ChoreoVariables.getPose("R1"));
    return Commands.runOnce(() -> drive.setPose(B.get()))
        .andThen(new StraightDriveToPose(drive, R1, 0.04));
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
            new StraightDriveToPose(drive, scorePoint, 0.04)
                .withTimeout(4)
                .andThen(fieldAlignment.alignToReef(left).withTimeout(5))
                .andThen(orchestrator.moveElevatorToLevel(false, 4).withTimeout(3))
                .andThen()
                // .andThen(orchestrator.placeCoral(4))
                .andThen(Commands.waitSeconds(1.2))
                .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)));
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
                .andThen(orchestrator.moveElevatorToLevel(false, 4).withTimeout(3))
                .andThen(coral.dumpCoral().withTimeout(4))
                // .andThen(orchestrator.placeCoral(4))
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
            new StraightDriveToPose(drive, scorePoint, 0.04)
                .withTimeout(4)
                .andThen(fieldAlignment.alignToReef(left))
                .withTimeout(5)
                .andThen(orchestrator.placeCoral(4))
                .andThen(Commands.waitSeconds(1.2))
                .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)));
  }

  public Command TwoCoralCScore(boolean left) {
    Supplier<Pose2d> C =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    ChoreoVariables.getPose("C").getTranslation(),
                    ChoreoVariables.getPose("C").getRotation().plus(Rotation2d.k180deg)));
    Supplier<Pose2d> scorePoint1 =
        () ->
            new Pose2d(
                new Translation2d(5.579622268676758, 2.2639400959014893),
                new Rotation2d(-0.982794168198375));
    return Commands.runOnce(() -> drive.setPose(C.get()))
        .andThen(Commands.waitSeconds(1))
        .andThen(
            new StraightDriveToPose(drive, () -> AllianceFlipUtil.apply(scorePoint1.get()), 0.04)
                .withTimeout(4)
                .andThen(fieldAlignment.alignToReef(left))
                .withTimeout(5)
                .andThen(orchestrator.placeCoral(4))
                .andThen(Commands.waitSeconds(1.2))
                .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)))
        .andThen(
            new StraightDriveToPose(
                    drive,
                    () ->
                        fieldAlignment
                            .getBranchPosition(false, fieldAlignment.closestReefFace())
                            .get()
                            .transformBy(new Transform2d(0, -3, new Rotation2d())),
                    0.04)
                .withTimeout(1)
                .andThen(fieldAlignment.alignToCoralStation())
                .withTimeout(4)
                .andThen(orchestrator.intake()))
        .andThen(
            new StraightDriveToPose(
                    drive, () -> AllianceFlipUtil.apply(new Pose2d(1, 1, new Rotation2d())), 0.04)
                .withTimeout(4)
                .andThen(
                    fieldAlignment
                        .alignToReef(left)
                        .withTimeout(4)
                        .andThen(orchestrator.placeCoral(4))
                        .andThen(Commands.waitSeconds(1.2))
                        .andThen(
                            orchestrator.moveElevatorToSetpoint(
                                ElevatorConstants.INTAKE_POSITION))));
  }

  public Command sigmaTwoScore(boolean left) {
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
        .andThen(new StraightDriveToPose(drive, scorePoint1, 0.8))
        .withTimeout(2.5)
        .andThen(fieldAlignment.alignToReef(left))
        .withTimeout(5)
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(/* 1.2 */ 0.3))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION))
        .andThen(new StraightDriveToPose(drive, scorePoint2, 0.8).withTimeout(2.5))
        // .andThen(Commands.waitSeconds(1.2))
        .andThen(fieldAlignment.alignToCoralStation().withTimeout(1))
        .andThen(orchestrator.intake().until(coral::hasCoral))
        .andThen(
            Commands.race(
                new StraightDriveToPose(drive, scorePoint3, 0.8)
                    .andThen(Commands.waitSeconds(0.5))
                    .andThen(
                        fieldAlignment
                            .alignToReef(left)
                            .withTimeout(2)
                            .andThen(Commands.waitSeconds(0.3 /*1.2 */))),
                coral.stop()))
        .andThen(orchestrator.placeCoral(4))
        .andThen(Commands.waitSeconds(1.2))
        .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION));
  }
}
