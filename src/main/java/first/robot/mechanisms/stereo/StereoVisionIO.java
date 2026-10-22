package first.robot.mechanisms.stereo;

import org.littletonrobotics.junction.AutoLog;

public interface StereoVisionIO {
  @AutoLog
  public static class StereoVisionIOInputs {
    public boolean leftConnected = false;
    public double[] leftYawDegrees = new double[0];
    public double[] leftPitchDegrees = new double[0];
    public double[] leftConfidence = new double[0];
    public double leftTimestamp = 0.0;

    public boolean rightConnected = false;
    public double[] rightYawDegrees = new double[0];
    public double[] rightPitchDegrees = new double[0];
    public double[] rightConfidence = new double[0];
    public double rightTimestamp = 0.0;
  }

  public default void updateInputs(StereoVisionIOInputs inputs) {}
}
