package frc.lib.constantcontrol;

import frc.lib.tuning.LoggedTunableNumber;

/** Simple constant wrapper for a ArmFeedforward controller */
public class LoggedArmFeedforwardConstant {
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kCos;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;

  /**
   * Configures a ArmFeedforward constant
   *
   * @param kS static gain
   * @param kCos gravity gain
   * @param kV velocity gain
   * @param kA acceleration gain
   */
  public LoggedArmFeedforwardConstant(
      double kS, double kCos, double kV, double kA, String dashboardKey) {
    this.kS = new LoggedTunableNumber(dashboardKey + "/kS", kS);
    this.kCos = new LoggedTunableNumber(dashboardKey + "/kCos", kCos);
    this.kV = new LoggedTunableNumber(dashboardKey + "/kV", kV);
    this.kA = new LoggedTunableNumber(dashboardKey + "/kA", kA);
  }

  /**
   * Configures a ArmFeedforward constant
   *
   * @param kS static gain
   * @param kCos gravity gain
   * @param kV velocity gain
   */
  public LoggedArmFeedforwardConstant(double kS, double kCos, double kV, String dashboardKey) {
    this(kS, kCos, kV, 0.0, dashboardKey);
  }

  /**
   * Get static gain
   *
   * @return static gain
   */
  public double getkS() {
    return kS.get();
  }

  /**
   * Get gravity gain
   *
   * @return gravity gain
   */
  public double getkCos() {
    return kCos.get();
  }

  /**
   * Get velocity gain
   *
   * @return velocity gain
   */
  public double getkV() {
    return kV.get();
  }

  /**
   * Get acceleration gain
   *
   * @return acceleration gain
   */
  public double getkA() {
    return kA.get();
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
    return kS.hasChanged(id) || kCos.hasChanged(id) || kV.hasChanged(id) || kA.hasChanged(id);
  }
}
