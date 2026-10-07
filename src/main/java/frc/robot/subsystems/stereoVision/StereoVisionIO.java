package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface StereoVisionIO {
  @AutoLog
  public static class StereoVisionIOInputs {
    public boolean connected = false;
    public int[] targetClassIds = new int[0];
    public double[] targetYawDegrees = new double[0];
    public double[] targetPitchDegrees = new double[0];
    public double timestampSeconds = 0.0;
  }

  Transform3d getRobotToCamera();

  default void updateInputs(StereoVisionIOInputs inputs) {}
}
