package frc.robot.subsystems;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
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
    private final MotorControllerEncoder spitterMotor;

    private final LinearSystem<N1, N1, N1> spitterPlant;
    private final KalmanFilter<N1, N1, N1> spitterObserver;
    private final LinearQuadraticRegulator<N1, N1, N1> spitterController;
    private final LinearSystemLoop<N1, N1, N1> spitterLoop;
    private final Timer timer;
    private boolean timerEnabled = false;

  /**
   * Returns calculated flywheel speed in rad/s from any distance in meters
   * @param distance Distance in meters
   * @return Calculated flywheel speed in rad/s
   */
  public static double getFlywheelVelocity(double distance) {
    return RobotConstants.get().spitter2022MaxVelocity() * ((RobotConstants.get().spitter2022DistanceLinearM() * distance) + RobotConstants.get().spitter2022DistanceLinearB());
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
        spitterMotor.setUnitsPerRotation(2.0 * Math.PI * RobotConstants.get().spitter2022GearRatio().getRotationsPerInput());

        spitterPlant = RobotConstants.get().spitter2022FF().getVelocityPlant();
        // TODO: Switch to flywheel plant, after gearing and moment of inertia are calculated
      //  spitterPlant =
      //          LinearSystemId.createFlywheelSystem(
      //                  DCMotor.getNEO(1), RobotConstants.get().spitter2022MomentOfInertia(), RobotConstants.get().spitter2022GearRatio().getRotationsPerOutput());
        spitterObserver =
            new KalmanFilter<>(
                    Nat.N1(),
                    Nat.N1(),
                    spitterPlant,
                    VecBuilder.fill(3.0), // How accurate we think our model is
                    VecBuilder.fill(0.01), // How accurate we think our encoder
                    // data is
                    0.020);

        spitterController =
                new LinearQuadraticRegulator<>(
                        spitterPlant,
                        VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
                        // this to more heavily penalize state excursion, or make the controller behave more
                        // aggressively.
                        VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
                        // heavily penalize control effort, or make the controller less aggressive. 12 is a good
                        // starting point because that is the (approximate) maximum voltage of a battery.
                        0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
        // lower if using notifiers.

        spitterLoop =
                new LinearSystemLoop<>(spitterPlant, spitterController, spitterObserver, 12.0, 0.020);

        spitterLoop.reset(VecBuilder.fill(spitterMotor.getVelocity()));

        timer = new Timer();
    }

    /**
     * Sets the speed of the motor, using velocity if it is enabled in the constants.
     *
     * @param speed the speed to set the motor to.
     */
    public void setSpeed(double speed) {
        setVelocity(speed * RobotConstants.get().spitter2022MaxVelocity());
    }

    /**
     * Sets the velocity for the motor, using the velocity controllers
     *
     * @param velocity the velocity to set the motor to.
     */
    public void setVelocity(double velocity) {
        spitterLoop.setNextR(VecBuilder.fill(velocity));
    }

    public void reset() {
        spitterLoop.reset(VecBuilder.fill(spitterMotor.getVelocity()));
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
      // System.out.printf("Setting velocity to %f rad/s, %f percent,l distance is %f meters, %n", getFlywheelVelocity(), getFlywheelVelocity()/RobotConstants.get().spitter2022MaxVelocity(), HubTarget.getDistance());
        setVelocity(getFlywheelVelocity());
    }

    /** Stop the flywheel. */
    public void stop() {
        spitterLoop.setNextR(VecBuilder.fill(0));
    }

    /**
     * Checks if the flywheel speed has reached the threshold.
     *
     * <p>If velocity is not used, return true as atSpeed can not be used.
     *
     * @return if the threshold has been met
     */
    public boolean isAtShootingSpeed() {
        boolean atSpeed = Math.abs(spitterLoop.getError(0)) <= SHOOTING_SPEED_TOLERANCE;
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
        spitterLoop.correct(VecBuilder.fill(spitterMotor.getVelocity()));
        spitterLoop.predict(0.020);
        spitterMotor.setVoltage(spitterLoop.getU(0));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Flywheel Velocity", spitterMotor::getVelocity, null);
        builder.addDoubleProperty(
                "Flywheel Velocity Error",
                () -> spitterLoop.getError(0),
                null);
    }
}
