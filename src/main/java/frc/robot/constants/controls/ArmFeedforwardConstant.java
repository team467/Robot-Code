package frc.robot.constants.controls;

import edu.wpi.first.math.controller.ArmFeedforward;

/** Simple constant wrapper for a ArmFeedforward controller */
public class ArmFeedforwardConstant {
  private final double kS;
  private final double kCos;
  private final double kV;
  private final double kA;

  /**
   * Configures a ArmFeedforward constant
   *
   * @param kS static gain
   * @param kCos gravity gain
   * @param kV velocity gain
   * @param kA acceleration gain
   */
  public ArmFeedforwardConstant(double kS, double kCos, double kV, double kA) {
    this.kS = kS;
    this.kCos = kCos;
    this.kV = kV;
    this.kA = kA;
  }

  /**
   * Configures a ArmFeedforward constant
   *
   * @param kS static gain
   * @param kCos gravity gain
   * @param kV velocity gain
   */
  public ArmFeedforwardConstant(double kS, double kCos, double kV) {
    this(kS, kCos, kV, 0);
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
  public double getkCos() {
    return kCos;
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
   * Creates a ArmFeedforward from constants given
   *
   * @return a ArmFeedforward
   */
  public ArmFeedforward getFeedforward() {
    return new ArmFeedforward(kS, kCos, kV, kA);
  }
}
