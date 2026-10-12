// THIS IS ONLY FOR LAPTOP BASED TESTING AND NOT ON THE ROBOT BUT SHOULDNT HRUT ANYTHING

package frc.robot.subsystems.stereoVision;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Locale;

// desktop-only test ccam 0 is left, camera 1 right both level and parallel.
public final class StereoVisionTableTest {
  private StereoVisionTableTest() {}

  public static void main(String[] args) throws InterruptedException {
    double baseline = 0.30;
    double duration = Double.POSITIVE_INFINITY;
    for (int i = 0; i < args.length; i++) {
      switch (args[i]) {
        case "--baseline" -> baseline = Double.parseDouble(args[++i]);
        case "--seconds" -> duration = Double.parseDouble(args[++i]);
        default -> throw new IllegalArgumentException("Use --baseline METERS [--seconds SECONDS]");
      }
    }
    if (!Double.isFinite(baseline)
        || baseline <= 0.0
        || Double.isNaN(duration)
        || duration <= 0.0) {
      throw new IllegalArgumentException("Spacing and duration must be positive.");
    }
    if (!HAL.initialize(500, 0))
      throw new IllegalStateException("Desktop HAL failed to initialize");
    var nt = NetworkTableInstance.getDefault();
    nt.startServer("", "0.0.0.0", 1735, 5810);
    System.out.printf(
        Locale.US, "Stereo table test: spacing %.3f m (measure before testing).%n", baseline);
    System.out.println(
        "Laptop is the NetworkTables server (TCP 5810). Set PhotonVision's server address to this laptop's network IPv4 address.");
    System.out.println(
        "Camera 0 LEFT, camera 1 RIGHT, level and parallel. Origin: midpoint of optical centers. X forward, Y left, Z up; meters.");
    System.out.println("Coordinates appear here. Ctrl+C stops. Start with one stationary ball.");
    var vision =
        new StereoVision(
            new StereoVisionIOPhotonVision(
                StereoVisionConstants.camera0Name,
                new Transform3d(new Translation3d(0, baseline / 2, 0), new Rotation3d())),
            new StereoVisionIOPhotonVision(
                StereoVisionConstants.camera1Name,
                new Transform3d(new Translation3d(0, -baseline / 2, 0), new Rotation3d())));
    long start = System.nanoTime();
    long nextPrint = start;
    try {
      while ((System.nanoTime() - start) / 1e9 < duration) {
        vision.refreshInputs();
        if (System.nanoTime() >= nextPrint) {
          System.out.println("Camera 0: " + vision.getCameraStatus(0));
          System.out.println("Camera 1: " + vision.getCameraStatus(1));
          var positions = vision.getFuelPositionsRobotRelative();
          if (positions.isEmpty()) {
            String reason = vision.getObservationStatus();
            System.out.println(
                "No valid position: "
                    + (reason.equals("Ready")
                        ? "rays do not match (check spacing, angles, camera order, shared visibility)"
                        : reason));
          } else {
            for (int i = 0; i < positions.size(); i++) {
              var p = positions.get(i);
              System.out.printf(
                  Locale.US,
                  "Fuel %d: X=%.3f m Y=%.3f m Z=%.3f m%n",
                  i,
                  p.getX(),
                  p.getY(),
                  p.getZ());
            }
          }
          nextPrint = System.nanoTime() + 500_000_000L;
        }
        Thread.sleep(20);
      }
    } finally {
      nt.stopServer();
      HAL.shutdown();
    }
  }
}
