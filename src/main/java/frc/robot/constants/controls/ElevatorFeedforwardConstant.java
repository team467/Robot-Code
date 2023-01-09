package frc.robot.constants.controls;

import edu.wpi.first.math.controller.ElevatorFeedforward;

/** Simple constant wrapper for a ElevatorFeedforward controller */
public class ElevatorFeedforwardConstant {
  private final double kS;
  private final double kG;
  private final double kV;
  private final double kA;

  /**
   * Configures a ElevatorFeedforward constant
   *
   * @param kS static gain
   * @param kG gravity gain
   * @param kV velocity gain
   * @param kA acceleration gain
   */
  public ElevatorFeedforwardConstant(double kS, double kG, double kV, double kA) {
    this.kS = kS;
    this.kG = kG;
    this.kV = kV;
    this.kA = kA;
  }

  /**
   * Configures a ElevatorFeedforward constant
   *
   * @param kS static gain
   * @param kG gravity gain
   * @param kV velocity gain
   */
  public ElevatorFeedforwardConstant(double kS, double kG, double kV) {
    this(kS, kG, kV, 0);
  }

  /**
   * Get static gain
   *
   * @return static gain
   */
  public double getkS() {
    return kS;
  }

  /**
   * Get gravity gain
   *
   * @return gravity gain
   */
  public double getkG() {
    return kG;
  }

  /**
   * Get velocity gain
   *
   * @return velocity gain
   */
  public double getkV() {
    return kV;
  }

  /**
   * Get acceleration gain
   *
   * @return acceleration gain
   */
  public double getkA() {
    return kA;
  }

  /**
   * Creates a ElevatorFeedforward from constants given
   *
   * @return a ElevatorFeedforward
   */
  public ElevatorFeedforward getFeedforward() {
    return new ElevatorFeedforward(kS, kG, kV, kA);
  }
}
