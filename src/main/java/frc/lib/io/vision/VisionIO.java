package frc.lib.io.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean hasTargets = false;
    public double bestYaw = 0.0;
    public double bestPitch = 0.0;
    public double bestArea = 0.0;
    public double bestSkew = 0.0;
    public long fiducialId = 0;
    /**
     * Translation3d formatted to
     * [(Translation)x,(Translation)y,(Translation)z,(Quaternion)w,(Quaternion)x,(Quaternion)y,(Quaternion)z],
     */
    public double[] bestBestCameraToTarget = new double[] {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    /**
     * Translation3d formatted to
     * [(Translation)x,(Translation)y,(Translation)z,(Quaternion)w,(Quaternion)x,(Quaternion)y,(Quaternion)z],
     */
    public double[] bestAltCameraToTarget = new double[] {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

    public double bestPoseAmbiguity = 0.0;
    /**
     * Pose3d formatted to
     * [(Translation)x,(Translation)y,(Translation)z,(Quaternion)w,(Quaternion)x,(Quaternion)y,(Quaternion)z],
     */
    public double[] estimatedPose = new double[] {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

    public double estimatedPoseTimestamp = -1;
  }

  default void updateInputs(VisionIOInputs inputs) {}
  default void setDriverMode(boolean driverMode) {}
  default void setPipelineIndex(int index) {}
}
