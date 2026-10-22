package first.robot.mechanisms.stereo;

import static first.robot.constants.FieldConstants.fieldLength;
import static first.robot.constants.FieldConstants.fieldWidth;
import static first.robot.mechanisms.stereo.StereoVisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;

public class StereoVisionIOSim extends StereoVisionIOPhotonVision {
  private final Supplier<Pose2d> poseSupplier;
  private final VisionSystemSim visionSim = new VisionSystemSim("stereo");
  private final PhotonCameraSim leftSim;
  private final PhotonCameraSim rightSim;
  private final Random random = new Random(simRandomSeed);
  private final List<VisionTargetSim> fuel = new ArrayList<>();

  public StereoVisionIOSim(Supplier<Pose2d> poseSupplier) {
    super(leftCameraName, rightCameraName);
    this.poseSupplier = poseSupplier;

    leftSim = cameraSim(leftCamera);
    rightSim = cameraSim(rightCamera);
    visionSim.addCamera(leftSim, robotToLeftCamera);
    visionSim.addCamera(rightSim, robotToRightCamera);
    scatterFuel();
  }

  private static PhotonCameraSim cameraSim(PhotonCamera camera) {
    var properties = new SimCameraProperties();
    properties.setCalibration(
        simResolutionWidth, simResolutionHeight, Rotation2d.fromDegrees(simDiagonalFovDegrees));
    var sim = new PhotonCameraSim(camera, properties);
    sim.enableRawStream(simEnableCameraStreams);
    sim.enableProcessedStream(simEnableCameraStreams);
    return sim;
  }

  public void scatterFuel() {
    visionSim.removeVisionTargets("fuel");
    fuel.clear();
    TargetModel model = new TargetModel(fuelDiameterMeters);
    double spanX = fieldLength - 2 * simFieldMarginMeters;
    double spanY = fieldWidth - 2 * simFieldMarginMeters;
    for (int i = 0; i < simFuelCount; i++) {
      double x = simFieldMarginMeters + random.nextDouble() * spanX;
      double y = simFieldMarginMeters + random.nextDouble() * spanY;
      Pose3d pose = new Pose3d(x, y, fuelDiameterMeters / 2.0, Rotation3d.ZERO);
      fuel.add(new VisionTargetSim(pose, model, fuelClassId, -1.0f));
    }
    visionSim.addVisionTargets("fuel", fuel.toArray(new VisionTargetSim[0]));
  }

  @Override
  public void updateInputs(StereoVisionIOInputs inputs) {
    Pose2d robotPose = poseSupplier.get();
    visionSim.update(robotPose);
    super.updateInputs(inputs);

    Pose3d robot = new Pose3d(robotPose);
    Pose3d leftPose = robot.transformBy(robotToLeftCamera);
    Pose3d rightPose = robot.transformBy(robotToRightCamera);
    List<Pose3d> all = new ArrayList<>();
    List<Pose3d> visible = new ArrayList<>();
    for (VisionTargetSim target : fuel) {
      all.add(target.getPose());
      if (leftSim.canSeeTargetPose(leftPose, target)
          && rightSim.canSeeTargetPose(rightPose, target)) {
        visible.add(target.getPose());
      }
    }
    Logger.recordOutput("StereoVision/Sim/AllFuel", all.toArray(new Pose3d[0]));
    Logger.recordOutput("StereoVision/Sim/VisibleFuel", visible.toArray(new Pose3d[0]));
  }
}
