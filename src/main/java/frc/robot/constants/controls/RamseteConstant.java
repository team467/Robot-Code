package frc.robot.constants.controls;

import edu.wpi.first.math.controller.RamseteController;

/**
 * Simple constant wrapper for a Ramsete controller
 *
 * <p>The Ramsete Controller is a trajectory tracker that is built in to WPILib.
 *
 * @see <a
 *     href="https://docs.wpilib.org/en/latest/docs/software/advanced-controls/trajectories/ramsete.html#constructing-the-ramsete-controller-object">...</a>
 */
public class RamseteConstant {
  private final double kB;
  private final double kZeta;

  public RamseteConstant(double kB, double kZeta) {
    this.kB = kB;
    this.kZeta = kZeta;
  }

  public RamseteConstant() {
    this(2.0, 0.7);
  }

  public double getkB() {
    return kB;
  }

  public double getkZeta() {
    return kZeta;
  }

  public RamseteController getController() {
    return new RamseteController(kB, kZeta);
  }
}
