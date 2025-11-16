package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auto.StraightDriveToPose;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.stereoVision.stereoVision;
import frc.robot.subsystems.stereoVision.stereoVisionIO.gamePieceType;
import frc.robot.subsystems.stereoVision.stereoVisionIO.objectObservation;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class gamePieceAlignment {

  private objectObservation closestGamepiece;
  @AutoLogOutput private Pose2d gamePiecePose;
  private final Drive drive;
  private final stereoVision stereoVision;

  public gamePieceAlignment(Drive drive, stereoVision stereoVision) {
    this.drive = drive;
    this.stereoVision = stereoVision;
  }

  public Command alignToNearestCoralPiece() {
    return Commands.defer(
        () -> new StraightDriveToPose(getClosestCoral().get(), drive), Set.of(drive));
  }

  public void findClosestCoral() {
    var coralDetected = stereoVision.getCoralObservations();
    if (!coralDetected.isEmpty()) {
      objectObservation closestCoral = new objectObservation(new Transform2d(), gamePieceType.NULL);
      for (var poseObs : coralDetected) {
        if (poseObs.pose().getTranslation().getNorm()
            < closestCoral.pose().getTranslation().getNorm()) {
          closestCoral = poseObs;
        }
      }
      closestGamepiece = closestCoral;
      gamePiecePose = drive.getPose().plus(closestCoral.pose().inverse());
    }
  }

  public Supplier<Pose2d> getClosestCoral() {
    return () -> {
      findClosestCoral();
      return gamePiecePose;
    };
  }

  public void findClosestAlgae() {
    var algaeDetected = stereoVision.getAlgaeObservations();
    if (!algaeDetected.isEmpty()) {
      objectObservation closestAlgae = new objectObservation(new Transform2d(), gamePieceType.NULL);
      for (var poseObs : algaeDetected) {
        if (poseObs.pose().getTranslation().getNorm()
            < closestAlgae.pose().getTranslation().getNorm()) {
          closestAlgae = poseObs;
        }
      }
      closestGamepiece = closestAlgae;
      gamePiecePose = drive.getPose().plus(closestAlgae.pose().inverse());
    }
  }
}
