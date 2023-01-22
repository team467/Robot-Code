package frc.lib.io.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean hasTargets = false;
    public double[] yaws = new double[20];
    public double[] pitches = new double[20];
    public double[] areas = new double[20];
    public double[] skews = new double[20];
    public long[] fiducialIds = new long[20];
    /**
     * Translation3d formatted to
     * [(boolean)valid,(Translation)x,(Translation)y,(Translation)z,(Quaternion)w,(Quaternion)x,(Quaternion)y,(Quaternion)z],
     */
    public double[] bestCameraToTargets = new double[10*8]; //new double[] {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    /**
     * Translation3d formatted to
     * [(boolean)valid,,(Translation)y,(Translation)z,(Quaternion)w,(Quaternion)x,(Quaternion)y,(Quaternion)z],
     */
    public double[] altCameraToTargets = new double[10*8]; //new double[] {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

    public double[] poseAmbiguities = new double[20];
    /**
     * Pose3d formatted to
     * [(boolean)valid,(Translation)x,(Translation)y,(Translation)z,(Quaternion)w,(Quaternion)x,(Quaternion)y,(Quaternion)z],
     */
    public double[] estimatedPose = new double[] {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

    public double estimatedPoseTimestamp = -1.0;
  }

  default void updateInputs(VisionIOInputs inputs) {}
  default void setDriverMode(boolean driverMode) {}
  default void setPipelineIndex(int index) {}
}
