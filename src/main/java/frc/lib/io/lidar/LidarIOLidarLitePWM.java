package frc.lib.io.lidar;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

public class LidarIOLidarLitePWM implements LidarIO {
  /**
   * Adjust the Calibration Offset to compensate for differences in each unit. We've found this is a
   * reasonably constant value for readings in the 25 cm to 600 cm range. You can also use the
   * offset to zero out the distance between the sensor and edge of the robot.
   */
  private static final int CALIBRATION_OFFSET = 0;

  private final Counter counter;
  private int printedWarningCount = 5;

  public LidarIOLidarLitePWM(int dioChannel) {
    counter = new Counter(new DigitalInput(dioChannel));
    counter.setMaxPeriod(1.0);
    // Configure for measuring rising to falling pulses
    counter.setSemiPeriodMode(true);
    counter.reset();
  }

  @Override
  public void updateInputs(LidarIOInputs inputs) {
    inputs.distance = getDistance();
  }

  private double getDistance() {
    double cm;
    /* If we haven't seen the first rising to falling pulse, then we have no measurement.
     * This happens when there is no LIDAR-Lite plugged in, btw.
     */
    if (counter.get() < 1 && printedWarningCount-- > 0) {
      DriverStation.reportError("[LidarLite] waiting for distance measurement", false);
    }
    /* getPeriod returns time in seconds. The hardware resolution is microseconds.
     * The LIDAR-Lite unit sends a high signal for 10 microseconds per cm of distance.
     */
    cm = (counter.getPeriod() * 1e6 / 10.0) + CALIBRATION_OFFSET;
    return cm / 100; // Meters
  }
}
