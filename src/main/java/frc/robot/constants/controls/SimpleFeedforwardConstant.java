package frc.robot.constants.controls;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

/** Simple constant wrapper for a SimpleFeedforward controller */
public class SimpleFeedforwardConstant {
  private final double kS;
  private final double kV;
  private final double kA;

  /**
   * Configures a SimpleFeedforward constant
   *
   * @param kS static gain
   * @param kV velocity gain
   * @param kA acceleration gain
   */
  public SimpleFeedforwardConstant(double kS, double kV, double kA) {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
  }

  /**
   * Configures a SimpleFeedforward constant
   *
   * @param kS static gain
   * @param kV velocity gain
   */
  public SimpleFeedforwardConstant(double kS, double kV) {
    this(kS, kV, 0);
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
   * Creates a SimpleMotorFeedforward from constants given
   *
   * @return a SimpleMotorFeedforward
   */
  public SimpleMotorFeedforward getFeedforward() {
    return new SimpleMotorFeedforward(kS, kV, kA);
  }

  // TODO: add javadoc comment for this
  public LinearSystem<N1, N1, N1> getVelocityPlant() {
    return LinearSystemId.identifyVelocitySystem(kV, kA);
  }

  // TODO: add javadoc comment for this
  public LinearSystem<N2, N1, N1> getPositionPlant() {
    return LinearSystemId.identifyPositionSystem(kV, kA);
  }
}
