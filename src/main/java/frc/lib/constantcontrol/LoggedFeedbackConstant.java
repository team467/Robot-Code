package frc.lib.constantcontrol;

import frc.lib.tuning.LoggedTunableNumber;

/** Simple constant wrapper for a PIDController controller */
public class LoggedFeedbackConstant {
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kD;

  /**
   * Configures a PIDController constant
   *
   * <p>The integral coefficient is not included as a proper feedforward should be used instead.
   *
   * @param kP proportional coefficient
   * @param kD derivative coefficient
   */
  public LoggedFeedbackConstant(double kP, double kD, String dashboardKey) {
    this.kP = new LoggedTunableNumber(dashboardKey + "/kP", kP);
    this.kD = new LoggedTunableNumber(dashboardKey + "/kD", kD);
  }

  /**
   * Configures a PIDController constant
   *
   * @param kP proportional coefficient
   */
  public LoggedFeedbackConstant(double kP, String dashboardKey) {
    this(kP, 0.0, dashboardKey);
  }

  /**
   * Get proportional coefficient
   *
   * @return proportional coefficient
   */
  public double getkP() {
    return kP.get();
  }

  /**
   * Get derivative coefficient
   *
   * @return derivative coefficient
   */
  public double getkD() {
    return kD.get();
  }

  /**
   * Checks whether the numbers have changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    return kP.hasChanged(id) || kD.hasChanged(id);
  }
}
