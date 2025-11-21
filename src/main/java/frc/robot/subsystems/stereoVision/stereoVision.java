package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.stereoVision.stereoVisionIO.gamePieceType;
import frc.robot.subsystems.stereoVision.stereoVisionIO.objectObservation;
import frc.robot.subsystems.stereoVision.stereoVisionIO.poseObservation;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class stereoVision extends SubsystemBase {
  private final stereoVisionIO io;
  private final Drive drive;
  private final stereoVisionInputsAutoLogged inputs;
  private final List<poseObservation> detectedObjects = new LinkedList<>();
  private final List<objectObservation> detectedObjectObservations = new LinkedList<>();
  private final List<Pose2d> objectPoses = new LinkedList<>();
  private List<objectObservation> coralObservations = new LinkedList<>();
  private List<objectObservation> algaeObservations = new LinkedList<>();
  @AutoLogOutput private Pose2d closestCoral = new Pose2d();
  @AutoLogOutput private Pose2d closestAlgae = new Pose2d();

  public stereoVision(stereoVisionIO io, Drive drive) {
    this.io = io;
    this.drive = drive;
    this.inputs = new stereoVisionInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("StereoVision", inputs);
    detectedObjects.clear();
    for (var transformations : inputs.objectObservations) {
      objectPoses.add((drive.getPose().transformBy(transformations.pose())));
      detectedObjects.add(
          new poseObservation(
              (drive.getPose().transformBy(transformations.pose())), transformations.type()));
    }
    coralObservations =
        Stream.of(inputs.objectObservations)
            .filter(objectObservation -> objectObservation.type() == gamePieceType.CORAL)
            .collect(Collectors.toList());
    algaeObservations =
        Stream.of(inputs.objectObservations)
            .filter(objectObservation -> objectObservation.type() == gamePieceType.ALGAE)
            .collect(Collectors.toList());
    objectObservation Coral = new objectObservation(new Transform2d(0,0,new Rotation2d()), gamePieceType.ALGAE);
    for (var poseObs : coralObservations) {
      if (poseObs.pose().getTranslation().getNorm()
          < Coral.pose().getTranslation().getNorm()) {
      Coral = poseObs;
      }
    }
    closestCoral = drive.getPose().plus(Coral.pose());
    objectObservation Algae = new objectObservation(new Transform2d(0,0,new Rotation2d()), gamePieceType.ALGAE);
    for (var poseObs : algaeObservations) {
      if (poseObs.pose().getTranslation().getNorm()
          < Algae.pose().getTranslation().getNorm()) {
        Algae = poseObs;
      }
    }
    closestAlgae = drive.getPose().plus(Algae.pose());

  }

  public List<poseObservation> getDetectedObjectsPoseObservations() {
    return detectedObjects;
  }

  public List<Pose2d> getDetectedObjectsPoses() {
    return objectPoses;
  }

  public List<objectObservation> getDetectedObjects() {
    return List.of(inputs.objectObservations);
  }

  public Pose2d getClosestCoral() {
    return closestCoral;
  }

  public Pose2d getClosestAlgae() {
    return closestAlgae;
  }

  public boolean seesGamePiece() {
    return inputs.seesGamePiece;
  }
}
