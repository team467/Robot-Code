package first.robot.mechanisms.vision;

import static first.robot.mechanisms.vision.VisionConstants.aprilTagLayout;

import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform3d;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final boolean stepsSim;

  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    stepsSim = visionSim == null;
    if (stepsSim) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    var cameraSim = new PhotonCameraSim(camera, new SimCameraProperties(), aprilTagLayout);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    if (stepsSim) {
      visionSim.update(poseSupplier.get());
    }
    super.updateInputs(inputs);
  }
}
