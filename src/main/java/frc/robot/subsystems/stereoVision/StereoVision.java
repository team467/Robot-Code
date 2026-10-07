package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class StereoVision extends SubsystemBase {
  private final StereoVisionIO[] io = new StereoVisionIO[2];
  private final StereoVisionIOInputsAutoLogged[] inputs =
      new StereoVisionIOInputsAutoLogged[2];
  private final Alert[] disconnectedAlerts = new Alert[2];

  public StereoVision() {
    this(
        new StereoVisionIOPhotonVision(
            StereoVisionConstants.camera0Name, StereoVisionConstants.robotToCamera0),
        new StereoVisionIOPhotonVision(
            StereoVisionConstants.camera1Name, StereoVisionConstants.robotToCamera1));
  }

  public StereoVision(StereoVisionIO camera0, StereoVisionIO camera1) {
    io[0] = camera0;
    io[1] = camera1;
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new StereoVisionIOInputsAutoLogged();
      disconnectedAlerts[i] =
          new Alert(
              "Stereo camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  public Rotation2d getTargetX(int cameraIndex) {
    int[] targetClassIds = inputs[cameraIndex].targetClassIds;
    double[] targetYawDegrees = inputs[cameraIndex].targetYawDegrees;
    for (int i = 0; i < targetClassIds.length; i++) {
      if (targetClassIds[i] == StereoVisionConstants.fuelObjectClassId) {
        return Rotation2d.fromDegrees(targetYawDegrees[i]);
      }
    }
    return new Rotation2d();
  }

  /** Returns triangulated robot-relative positions for detected fuel objects. */
  public List<Translation3d> getFuelPositionsRobotRelative() {
    var camera0 = inputs[0];
    var camera1 = inputs[1];
    if (camera0.targetClassIds.length == 0 || camera1.targetClassIds.length == 0) {
      return List.of();
    }

    double now = Timer.getFPGATimestamp();
    if (now - camera0.timestampSeconds > StereoVisionConstants.maxObservationAgeSeconds
        || now - camera1.timestampSeconds > StereoVisionConstants.maxObservationAgeSeconds
        || Math.abs(camera0.timestampSeconds - camera1.timestampSeconds)
            > StereoVisionConstants.maxTimestampDifferenceSeconds) {
      return List.of();
    }

    List<RayObservation> camera0Rays = getFuelRays(0);
    List<RayObservation> camera1Rays = getFuelRays(1);
    List<TriangulationCandidate> candidates = new ArrayList<>();
    for (RayObservation ray0 : camera0Rays) {
      for (RayObservation ray1 : camera1Rays) {
        triangulate(ray0, ray1).ifPresent(candidates::add);
      }
    }
    candidates.sort(Comparator.comparingDouble(TriangulationCandidate::raySeparation));

    Set<Integer> usedCamera0Targets = new HashSet<>();
    Set<Integer> usedCamera1Targets = new HashSet<>();
    List<Translation3d> positions = new ArrayList<>();
    for (TriangulationCandidate candidate : candidates) {
      if (usedCamera0Targets.contains(candidate.camera0TargetIndex())
          || usedCamera1Targets.contains(candidate.camera1TargetIndex())) {
        continue;
      }
      usedCamera0Targets.add(candidate.camera0TargetIndex());
      usedCamera1Targets.add(candidate.camera1TargetIndex());
      positions.add(candidate.position());
    }
    return List.copyOf(positions);
  }

  /** Returns the first triangulated fuel position for callers that only need one target. */
  public Optional<Translation3d> getTargetPositionRobotRelative() {
    return getFuelPositionsRobotRelative().stream().findFirst();
  }

  private List<RayObservation> getFuelRays(int cameraIndex) {
    StereoVisionIOInputsAutoLogged cameraInputs = inputs[cameraIndex];
    Transform3d robotToCamera = io[cameraIndex].getRobotToCamera();
    List<RayObservation> rays = new ArrayList<>();
    for (int i = 0; i < cameraInputs.targetClassIds.length; i++) {
      if (cameraInputs.targetClassIds[i] != StereoVisionConstants.fuelObjectClassId) {
        continue;
      }
      Translation3d direction =
          new Translation3d(1.0, 0.0, 0.0)
              .rotateBy(
                  robotToCamera
                      .getRotation()
                      .rotateBy(
                          new Rotation3d(
                              0.0,
                              Math.toRadians(cameraInputs.targetPitchDegrees[i]),
                              Math.toRadians(cameraInputs.targetYawDegrees[i]))));
      rays.add(new RayObservation(i, robotToCamera.getTranslation(), direction));
    }
    return rays;
  }

  private Optional<TriangulationCandidate> triangulate(
      RayObservation ray0, RayObservation ray1) {
    Translation3d origin0 = ray0.origin();
    Translation3d origin1 = ray1.origin();
    Translation3d direction0 = ray0.direction();
    Translation3d direction1 = ray1.direction();

    Translation3d betweenOrigins = origin0.minus(origin1);
    double dot = direction0.dot(direction1);
    double denominator = 1.0 - dot * dot;
    if (denominator < StereoVisionConstants.minimumTriangulationDenominator) {
      return Optional.empty();
    }

    double origin0Projection = direction0.dot(betweenOrigins);
    double origin1Projection = direction1.dot(betweenOrigins);
    double distance0 = (dot * origin1Projection - origin0Projection) / denominator;
    double distance1 = (origin1Projection - dot * origin0Projection) / denominator;
    if (distance0 <= 0.0 || distance1 <= 0.0) {
      return Optional.empty();
    }

    Translation3d point0 = origin0.plus(direction0.times(distance0));
    Translation3d point1 = origin1.plus(direction1.times(distance1));
    double raySeparation = point0.getDistance(point1);
    if (raySeparation > StereoVisionConstants.maximumRaySeparationMeters) {
      return Optional.empty();
    }
    return Optional.of(
        new TriangulationCandidate(
            ray0.targetIndex(),
            ray1.targetIndex(),
            point0.plus(point1).div(2.0),
            raySeparation));
  }

  private record RayObservation(
      int targetIndex, Translation3d origin, Translation3d direction) {}

  private record TriangulationCandidate(
      int camera0TargetIndex,
      int camera1TargetIndex,
      Translation3d position,
      double raySeparation) {}

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("StereoVision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
    }

    List<Translation3d> fuelPositions = getFuelPositionsRobotRelative();
    double[] coordinatesMeters = new double[fuelPositions.size() * 3];
    for (int i = 0; i < fuelPositions.size(); i++) {
      Translation3d position = fuelPositions.get(i);
      coordinatesMeters[i * 3] = position.getX();
      coordinatesMeters[i * 3 + 1] = position.getY();
      coordinatesMeters[i * 3 + 2] = position.getZ();
    }
    Logger.recordOutput("StereoVision/FuelPositionCount", fuelPositions.size());
    Logger.recordOutput("StereoVision/FuelPositionsRobotRelativeMeters", coordinatesMeters);
  }
}
