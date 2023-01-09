package frc.robot.constants.controls;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Simple constant wrapper for a PIDController controller */
public class FeedbackConstant {
  private final double kP;
  private final double kD;

  /**
   * Configures a PIDController constant
   *
   * <p>The integral coefficient is not included as a proper feedforward should be used instead.
   *
   * @param kP proportional coefficient
   * @param kD derivative coefficient
   */
  public FeedbackConstant(double kP, double kD) {
    this.kP = kP;
    this.kD = kD;
  }

  /**
   * Configures a PIDController constant
   *
   * @param kP proportional coefficient
   */
  public FeedbackConstant(double kP) {
    this(kP, 0);
  }

  /**
   * Get proportional coefficient
   *
   * @return proportional coefficient
   */
  public double getkP() {
    return kP;
  }

  /**
   * Get derivative coefficient
   *
   * @return derivative coefficient
   */
  public double getkD() {
    return kD;
  }

  /**
   * Creates a PIDController from constants given
   *
   * @return a PIDController
   */
  public PIDController getPIDController() {
    return new PIDController(kP, 0, kD);
  }

  /**
   * Creates a ProfiledPIDController, using the constants along with given constraints
   *
   * @param constraint constraints to add to ProfiledPIDController
   * @return a ProfiledPIDController
   */
  public ProfiledPIDController getProfiledPIDController(TrapezoidProfile.Constraints constraint) {
    return new ProfiledPIDController(kP, 0, kD, constraint);
  }
}
