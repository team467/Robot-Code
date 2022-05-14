package frc.robot.subsystems;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
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

  private final double SHOOTING_SPEED_TOLERANCE = 1.0;
  private final MotorControllerEncoder bottomSpitterMotor;
  private final LinearSystem<N1, N1, N1> bottomSpitterPlant;
  private final KalmanFilter<N1, N1, N1> bottomSpitterObserver;
  private final LinearQuadraticRegulator<N1, N1, N1> bottomSpitterController;
  private final LinearSystemLoop<N1, N1, N1> bottomSpitterLoop;

  private final MotorControllerEncoder topSpitterMotor;
  private final LinearSystem<N1, N1, N1> topSpitterPlant;
  private final KalmanFilter<N1, N1, N1> topSpitterObserver;
  private final LinearQuadraticRegulator<N1, N1, N1> topSpitterController;
  private final LinearSystemLoop<N1, N1, N1> topSpitterLoop;

  private final Timer timer;
  private boolean timerEnabled = false;

  /**
   * Returns calculated bottom flywheel speed in rad/s from any distance in meters
   *
   * @param distance Distance in meters
   * @return Calculated flywheel speed in rad/s
   */
  public static double getBottomFlywheelVelocity(double distance) {
    return ((RobotConstants.get().bottomSpitter2022DistanceLinearM() * distance)
        + RobotConstants.get().bottomSpitter2022DistanceLinearB());
  }

  /**
   * Returns calculated top flywheel speed in rad/s from any distance in meters
   *
   * @param distance Distance in meters
   * @return Calculated flywheel speed in rad/s
   */
  public static double getTopFlywheelVelocity(double distance) {
    return ((RobotConstants.get().topSpitter2022DistanceLinearM() * distance)
        + RobotConstants.get().topSpitter2022DistanceLinearB());
  }

  /**
   * Returns calculated bottom flywheel speed in rad/s from the distance to the target
   *
   * @return Calculated flywheel speed in rad/s
   */
  public static double getBottomFlywheelVelocity() {
    return getBottomFlywheelVelocity(HubTarget.getDistance());
  }

  /**
   * Returns calculated top flywheel speed in rad/s from the distance to the target
   *
   * @return Calculated flywheel speed in rad/s
   */
  public static double getTopFlywheelVelocity() {
    return getTopFlywheelVelocity(HubTarget.getDistance());
  }

  /** The spitter subsystem, contains the flywheel and its motor only. */
  public Spitter2022() {
    super();

    bottomSpitterMotor =
        MotorControllerFactory.create(
            RobotConstants.get().bottomSpitter2022MotorId(), MotorType.SPARK_MAX_BRUSHLESS);
    bottomSpitterMotor.setInverted(RobotConstants.get().bottomSpitter2022MotorInverted());
    bottomSpitterMotor.setUnitsPerRotation(
        2.0 * Math.PI * RobotConstants.get().bottomSpitter2022GearRatio().getRotationsPerInput());

    bottomSpitterPlant = RobotConstants.get().bottomSpitter2022FF().getVelocityPlant();
    bottomSpitterObserver =
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            bottomSpitterPlant,
            VecBuilder.fill(3.0), // How accurate we think our model is
            VecBuilder.fill(0.01), // How accurate we think our encoder
            // data is
            0.020);

    bottomSpitterController =
        new LinearQuadraticRegulator<>(
            bottomSpitterPlant,
            VecBuilder.fill(
                8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
            // this to more heavily penalize state excursion, or make the controller behave more
            // aggressively.
            VecBuilder.fill(
                12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
    // lower if using notifiers.

    bottomSpitterLoop =
        new LinearSystemLoop<>(bottomSpitterPlant, bottomSpitterController, bottomSpitterObserver, 12.0, 0.020);

    bottomSpitterLoop.reset(VecBuilder.fill(bottomSpitterMotor.getVelocity()));


    topSpitterMotor =
        MotorControllerFactory.create(
            RobotConstants.get().topSpitter2022MotorId(), MotorType.SPARK_MAX_BRUSHLESS);
    topSpitterMotor.setInverted(RobotConstants.get().topSpitter2022MotorInverted());
    topSpitterMotor.setUnitsPerRotation(
        2.0 * Math.PI * RobotConstants.get().topSpitter2022GearRatio().getRotationsPerInput());

    topSpitterPlant = RobotConstants.get().topSpitter2022FF().getVelocityPlant();
    topSpitterObserver =
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            topSpitterPlant,
            VecBuilder.fill(3.0), // How accurate we think our model is
            VecBuilder.fill(0.01), // How accurate we think our encoder
            // data is
            0.020);

    topSpitterController =
        new LinearQuadraticRegulator<>(
            topSpitterPlant,
            VecBuilder.fill(
                8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
            // this to more heavily penalize state excursion, or make the controller behave more
            // aggressively.
            VecBuilder.fill(
                12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
    // lower if using notifiers.

    topSpitterLoop =
        new LinearSystemLoop<>(topSpitterPlant, topSpitterController, topSpitterObserver, 12.0, 0.020);

    topSpitterLoop.reset(VecBuilder.fill(topSpitterMotor.getVelocity()));

    timer = new Timer();
  }

  /**
   * Sets the speed of the bottom motor, using velocity if it is enabled in the constants.
   *
   * @param speed the speed to set the motor to.
   */
  public void setBottomSpeed(double speed) {
    setBottomVelocity(speed * RobotConstants.get().bottomSpitter2022MaxVelocity());
  }

  /**
   * Sets the speed of the top motor, using velocity if it is enabled in the constants.
   *
   * @param speed the speed to set the motor to.
   */
  public void setTopSpeed(double speed) {
    setTopVelocity(speed * RobotConstants.get().topSpitter2022MaxVelocity());
  }

  /**
   * Sets the velocity for the bottom motor, using the velocity controllers
   *
   * @param velocity the velocity to set the motor to.
   */
  public void setBottomVelocity(double velocity) {
    bottomSpitterLoop.setNextR(VecBuilder.fill(velocity));
  }

  /**
   * Sets the velocity for the top motor, using the velocity controllers
   *
   * @param velocity the velocity to set the motor to.
   */
  public void setTopVelocity(double velocity) {
    topSpitterLoop.setNextR(VecBuilder.fill(velocity));
  }

  public void reset() {
    bottomSpitterLoop.reset(VecBuilder.fill(bottomSpitterMotor.getVelocity()));
    topSpitterLoop.reset(VecBuilder.fill(topSpitterMotor.getVelocity()));
  }

  /** Start spinning the flywheel. */
  public void forward() {
    setBottomSpeed(RobotConstants.get().bottomSpitter2022ForwardSpeed());
    setTopSpeed(RobotConstants.get().topSpitter2022ForwardSpeed());
  }

  /** Start spinning the flywheel backwards. */
  public void backward() {
    setBottomSpeed(-RobotConstants.get().bottomSpitter2022BackwardSpeed());
    setTopVelocity(-RobotConstants.get().topSpitter2022BackwardSpeed());
  }

  public void setSpitterToTarget() {
    // System.out.printf("Setting velocity to %f rad/s, %f percent,l distance is %f meters, %n",
    // getBottomFlywheelVelocity(), getBottomFlywheelVelocity()/RobotConstants.get().bottomSpitter2022MaxVelocity(),
    // HubTarget.getDistance());
    setBottomVelocity(getBottomFlywheelVelocity());
    setTopVelocity(getTopFlywheelVelocity());
  }

  /** Stop the flywheel. */
  public void stop() {
    bottomSpitterLoop.setNextR(VecBuilder.fill(0));
    topSpitterLoop.setNextR(VecBuilder.fill(0));
  }

  /**
   * Checks if the flywheel speed has reached the threshold.
   *
   * <p>If velocity is not used, return true as atSpeed can not be used.
   *
   * @return if the threshold has been met
   */
  public boolean isAtShootingSpeed() {
    boolean atSpeed = Math.abs(bottomSpitterLoop.getError(0)) <= SHOOTING_SPEED_TOLERANCE;
    if (atSpeed) {
      timer.start();
      timerEnabled = true;
    } else {
      timer.reset();
      timer.stop();
    }

    if (timer.get() > 0.1 && atSpeed) {
      timerEnabled = false;
      return true;
    }

    return false;
  }

  @Override
  public void periodic() {
    super.periodic();
    bottomSpitterLoop.correct(VecBuilder.fill(bottomSpitterMotor.getVelocity()));
    bottomSpitterLoop.predict(0.020);
    bottomSpitterMotor.setVoltage(bottomSpitterLoop.getU(0));
    topSpitterLoop.correct(VecBuilder.fill(topSpitterMotor.getVelocity()));
    topSpitterLoop.predict(0.020);
    topSpitterMotor.setVoltage(topSpitterLoop.getU(0));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Bottom Flywheel Velocity", bottomSpitterMotor::getVelocity, null);
    builder.addDoubleProperty("Top Flywheel Velocity", topSpitterMotor::getVelocity, null);
    builder.addDoubleProperty("Bottom Flywheel Velocity Error", () -> bottomSpitterLoop.getError(0), null);
    builder.addDoubleProperty("Top Flywheel Velocity Error", () -> topSpitterLoop.getError(0), null);
  }
}
