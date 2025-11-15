package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class stereoVision extends SubsystemBase {
  private final stereoVisionIO io;
  private final Drive drive;
  private final stereoVisionInputsAutoLogged inputs;
  private final List<Pose2d> detectedObjects = new LinkedList<>();

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
    for (var tranformations : inputs.poseObservations) {
      detectedObjects.add(drive.getPose().transformBy(tranformations.pose()));
    }
  }
}
