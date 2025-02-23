package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.lib.utils.ChoreoVariables;
import frc.robot.Orchestrator;
import frc.robot.commands.drive.FieldAlignment;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorConstants;
import java.util.function.Supplier;

public class AutosAlternate {
  private final Drive drive;
  private final Orchestrator orchestrator;
  private final FieldAlignment fieldAlignment;

  public AutosAlternate(Drive drive, Orchestrator orchestrator, FieldAlignment fieldAlignment) {
    this.drive = drive;
    this.orchestrator = orchestrator;
    this.fieldAlignment = fieldAlignment;
  }

  public Command zeroPiece() {
    Supplier<Pose2d> B = () -> AllianceFlipUtil.apply(ChoreoVariables.getPose("B"));
    return Commands.runOnce(() -> drive.setPose(B.get()))
        .andThen(new StraightDriveToPose(-1, 0, 0, drive));
  }

  public Command B1L() {
    Supplier<Pose2d> B = () -> AllianceFlipUtil.apply(ChoreoVariables.getPose("B"));
    Supplier<Pose2d> L1 = () -> AllianceFlipUtil.apply(ChoreoVariables.getPose("L1"));
    return Commands.runOnce(() -> drive.setPose(B.get()))
        .andThen(new StraightDriveToPose(drive, L1));
  }

  public Command B1R() {
    Supplier<Pose2d> B = () -> AllianceFlipUtil.apply(ChoreoVariables.getPose("B"));
    Supplier<Pose2d> R1 = () -> AllianceFlipUtil.apply(ChoreoVariables.getPose("R1"));
    return Commands.runOnce(() -> drive.setPose(B.get()))
        .andThen(new StraightDriveToPose(drive, R1));
  }

  public Command AScore() {
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
        .andThen(
            new StraightDriveToPose(drive, scorePoint)
                .andThen(fieldAlignment.alignToReef(false))
                .andThen(orchestrator.placeCoral(4)));
  }

  public Command BScore() {
    Supplier<Pose2d> B =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    ChoreoVariables.getPose("B").getTranslation(),
                    ChoreoVariables.getPose("B").getRotation().plus(Rotation2d.k180deg)));
    return Commands.runOnce(() -> drive.setPose(B.get()))
        .andThen(
            fieldAlignment
                .alignToReef(true)
                .andThen(orchestrator.placeCoral(4))
                .andThen(orchestrator.moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)));
  }

  public Command CScore() {
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
        .andThen(
            new StraightDriveToPose(drive, scorePoint)
                .andThen(fieldAlignment.alignToReef(false))
                .andThen(orchestrator.placeCoral(4)));
  }
}
