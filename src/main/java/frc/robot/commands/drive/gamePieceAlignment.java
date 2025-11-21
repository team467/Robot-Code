package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auto.StraightDriveToPose;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.stereoVision.stereoVision;
import frc.robot.subsystems.stereoVision.stereoVisionIO.objectObservation;
import java.util.Set;
import java.util.function.Supplier;

public class gamePieceAlignment {

  private objectObservation closestGamepiece;

  private final Drive drive;
  private final stereoVision stereoVision;

  public gamePieceAlignment(Drive drive, stereoVision stereoVision) {
    this.drive = drive;
    this.stereoVision = stereoVision;
  }

  public Command searchForGamePiece() {
    return Commands.run(
        () ->
            new StraightDriveToPose(drive.getPose().rotateBy(Rotation2d.k180deg), drive)
                .until(stereoVision::seesGamePiece));
  }

  public Command alignToNearestCoralPiece() {
    return Commands.defer(
        () -> new StraightDriveToPose(getClosestCoral().get(), drive), Set.of(drive));
  }

  public Command alignToNearestAlgaePiece() {
    return Commands.defer(
        () -> new StraightDriveToPose(getClosestAlgae().get(), drive), Set.of(drive));
  }

  public Supplier<Pose2d> getClosestCoral() {
    return stereoVision::getClosestCoral;
  }

  public Supplier<Pose2d> getClosestAlgae() {
    return stereoVision::getClosestAlgae;
  }
}
