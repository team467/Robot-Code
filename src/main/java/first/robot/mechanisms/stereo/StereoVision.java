package first.robot.mechanisms.stereo;

import static first.robot.mechanisms.stereo.StereoVisionConstants.*;

import first.robot.mechanisms.stereo.StereoTriangulation.Ray;
import first.robot.mechanisms.stereo.StereoTriangulation.Result;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;

public class StereoVision {
  private static final double MAX_FRAME_SKEW_SECONDS = 0.05;

  private record Match(int left, int right, Result result) {}

  private final StereoVisionIO io;
  private final StereoVisionIOInputsAutoLogged inputs = new StereoVisionIOInputsAutoLogged();
  private final Supplier<Pose2d> robotPoseSupplier;

  private List<Pose3d> fuelRobotRelative = List.of();
  private List<Pose3d> fuelField = List.of();

  public StereoVision(StereoVisionIO io, Supplier<Pose2d> robotPoseSupplier) {
    this.io = io;
    this.robotPoseSupplier = robotPoseSupplier;
    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  private void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("StereoVision", inputs);

    List<Match> matches = new ArrayList<>();
    if (Math.abs(inputs.leftTimestamp - inputs.rightTimestamp) <= MAX_FRAME_SKEW_SECONDS) {
      matches = matchDetections();
    }

    Pose3d robot = new Pose3d(robotPoseSupplier.get());
    List<Pose3d> robotRelative = new ArrayList<>();
    List<Pose3d> field = new ArrayList<>();
    double[] miss = new double[matches.size()];
    for (int i = 0; i < matches.size(); i++) {
      Result result = matches.get(i).result();
      robotRelative.add(new Pose3d(result.point(), Rotation3d.ZERO));
      field.add(robot.transformBy(new Transform3d(result.point(), Rotation3d.ZERO)));
      miss[i] = result.missDistanceMeters();
    }
    fuelRobotRelative = robotRelative;
    fuelField = field;

    Logger.recordOutput("StereoVision/FuelCount", fuelRobotRelative.size());
    Logger.recordOutput("StereoVision/RayMissMeters", miss);
    Logger.recordOutput("StereoVision/FuelRobotRelative", fuelRobotRelative.toArray(new Pose3d[0]));
    Logger.recordOutput("StereoVision/FuelField", fuelField.toArray(new Pose3d[0]));
    Optional<Pose3d> closest = getClosestFuelRobotRelative();
    Logger.recordOutput(
        "StereoVision/ClosestFuelRobotRelative",
        closest.map(p -> new Pose3d[] {p}).orElse(new Pose3d[0]));
    closest.ifPresent(
        p -> {
          Logger.recordOutput(
              "StereoVision/ClosestFuelDistanceMeters",
              p.getTranslation().toTranslation2d().getNorm());
          Logger.recordOutput(
              "StereoVision/ClosestFuelBearingDegrees",
              Math.toDegrees(Math.atan2(p.getY(), p.getX())));
        });
  }

  private List<Match> matchDetections() {
    int leftCount = Math.min(inputs.leftYawDegrees.length, inputs.leftPitchDegrees.length);
    int rightCount = Math.min(inputs.rightYawDegrees.length, inputs.rightPitchDegrees.length);
    List<Match> candidates = new ArrayList<>();
    for (int l = 0; l < leftCount; l++) {
      Ray left =
          StereoTriangulation.rayInRobotFrame(
              robotToLeftCamera, inputs.leftYawDegrees[l], inputs.leftPitchDegrees[l]);
      for (int r = 0; r < rightCount; r++) {
        Ray right =
            StereoTriangulation.rayInRobotFrame(
                robotToRightCamera, inputs.rightYawDegrees[r], inputs.rightPitchDegrees[r]);
        Optional<Result> result = StereoTriangulation.triangulate(left, right);
        if (result.isPresent() && isPlausible(result.get())) {
          candidates.add(new Match(l, r, result.get()));
        }
      }
    }
    candidates.sort(Comparator.comparingDouble(m -> matchCost(m.result())));

    boolean[] leftUsed = new boolean[leftCount];
    boolean[] rightUsed = new boolean[rightCount];
    List<Match> matches = new ArrayList<>();
    for (Match candidate : candidates) {
      if (!leftUsed[candidate.left()] && !rightUsed[candidate.right()]) {
        leftUsed[candidate.left()] = true;
        rightUsed[candidate.right()] = true;
        matches.add(candidate);
      }
    }
    return matches;
  }

  private static double heightError(Result result) {
    return Math.abs(result.point().getZ() - fuelDiameterMeters / 2.0);
  }

  private static double matchCost(Result result) {
    return result.missDistanceMeters() + heightError(result);
  }

  private static boolean isPlausible(Result result) {
    return result.missDistanceMeters() <= maxRayMissMeters
        && result.point().getNorm() <= maxRangeMeters
        && heightError(result) <= maxHeightErrorMeters;
  }

  public List<Pose3d> getFuelRobotRelative() {
    return fuelRobotRelative;
  }

  public List<Pose3d> getFuelField() {
    return fuelField;
  }

  public Optional<Pose3d> getClosestFuelRobotRelative() {
    return fuelRobotRelative.stream()
        .min(Comparator.comparingDouble(p -> p.getTranslation().toTranslation2d().getNorm()));
  }
}
