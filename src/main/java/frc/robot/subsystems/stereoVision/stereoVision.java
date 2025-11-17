package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Pose2d;
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
  @AutoLogOutput private final List<Pose2d> objectPoses = new LinkedList<>();
  @AutoLogOutput private List<objectObservation> coralObservations = new LinkedList<>();
  @AutoLogOutput private List<objectObservation> algaeObservations = new LinkedList<>();

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
      objectPoses.add(
          (drive.getPose().transformBy(transformations.pose())));
      detectedObjects.add(
          new poseObservation(
              (drive.getPose().transformBy(transformations.pose())),
              transformations.type()));
    }
    coralObservations =
        Stream.of(inputs.objectObservations)
            .filter(objectObservation -> objectObservation.type() == gamePieceType.CORAL)
            .collect(Collectors.toList());
    algaeObservations =
        Stream.of(inputs.objectObservations)
            .filter(objectObservation -> objectObservation.type() == gamePieceType.ALGAE)
            .collect(Collectors.toList());
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

  public List<objectObservation> getCoralObservations() {
    return coralObservations;
  }

  public List<objectObservation> getAlgaeObservations() {
    return algaeObservations;
  }

  public boolean seesGamePiece() {
    return inputs.seesGamePiece;
  }
}
