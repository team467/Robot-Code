package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.logging.RobotLogManager;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.MotorType;
import frc.robot.vision.HubTarget;
import org.apache.logging.log4j.Logger;

/** The spitter subsystem, contains the flywheel and its motor only. */
public class Spitter2022 extends SubsystemBase {

  private static final Logger LOGGER = RobotLogManager.getMainLogger(Spitter2022.class.getName());

  private final double SHOOTING_SPEED_TOLERANCE = 5.0;
  private final MotorControllerEncoder spitterMotor;
  private final PIDController spitterPIDController;
  private final SimpleMotorFeedforward spitterFFController;

  /**
   * Returns calculated flywheel speed in rad/s from any distance in meters
   * @param distance Distance in meters
   * @return Calculated flywheel speed in rad/s
   */
  public static double getFlywheelVelocity(double distance) {
    return 108.5792974 + (141.8090384 * distance);
  }

  /**
   * Returns calculated flywheel speed in rad/s from the distance to the target
   * @return Calculated flywheel speed in rad/s
   */
  public static double getFlywheelVelocity() {
    return getFlywheelVelocity(HubTarget.getDistance());
  }

  /** The spitter subsystem, contains the flywheel and its motor only. */
  public Spitter2022() {
    super();

    spitterMotor =
        MotorControllerFactory.create(
            RobotConstants.get().spitter2022MotorId(), MotorType.SPARK_MAX_BRUSHLESS);
    spitterMotor.setInverted(RobotConstants.get().spitter2022MotorInverted());

    // Feedback controller
    spitterPIDController = RobotConstants.get().spitter2022FB().getPIDController();
    // Feedforward controller
    spitterFFController = RobotConstants.get().spitter2022FF().getFeedforward();
  }

  /**
   * Sets the speed of the motor, using velocity if it is enabled in the constants.
   *
   * @param speed the speed to set the motor to.
   */
  public void setSpeed(double speed) {
    if (RobotConstants.get().spitter2022UseVelocity()) {
      LOGGER.debug(
          "Setting speed to "
              + speed
              + " using velocity, setting velocity to "
              + speed * RobotConstants.get().spitter2022MaxVelocity());
      setVelocity(speed * RobotConstants.get().spitter2022MaxVelocity());
    } else {
      LOGGER.debug("Setting speed to " + speed + " without using velocity");
      spitterMotor.set(speed);
    }
  }

  /**
   * Sets the velocity for the motor, using the velocity controllers
   *
   * @param velocity the velocity to set the motor to.
   */
  public void setVelocity(double velocity) {
    double output = spitterFFController.calculate(velocity);
    spitterPIDController.setSetpoint(velocity);
    if (RobotConstants.get().spitter2022UsePID()) {
      output += spitterPIDController.calculate(spitterMotor.getVelocity());
    }
    LOGGER.debug("Setting voltage to {}", output);
    spitterMotor.setVoltage(output);
  }

  /** Start spinning the flywheel. */
  public void forward() {
    setSpeed(RobotConstants.get().spitter2022ForwardSpeed());
  }

  /** Start spinning the flywheel backwards. */
  public void backward() {
    setSpeed(-RobotConstants.get().spitter2022BackwardSpeed());
  }

  public void setSpitterToTarget() {
    setVelocity(getFlywheelVelocity() / (2.0 * Math.PI));
  }

  /** Stop the flywheel. */
  public void stop() {
    spitterMotor.set(0.0);
  }

  /**
   * Checks if the flywheel speed has reached the threshold.
   *
   * <p>If velocity is not used, return true as atSpeed can not be used.
   *
   * @return if the threshold has been met
   */
  public boolean isAtShootingSpeed() {
    if (!RobotConstants.get().spitter2022UseVelocity()) {
      return true;
    }
    LOGGER.debug(
        "is "
            + Math.abs(spitterMotor.getVelocity() - spitterPIDController.getSetpoint())
            + " less than "
            + SHOOTING_SPEED_TOLERANCE
            + "?");
    return Math.abs(spitterMotor.getVelocity() - spitterPIDController.getSetpoint())
        <= SHOOTING_SPEED_TOLERANCE;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Flywheel Velocity", spitterMotor::getVelocity, null);
    builder.addDoubleProperty(
        "Flywheel Velocity Error",
        () -> spitterPIDController.getSetpoint() - spitterMotor.getVelocity(),
        null);
  }
}
