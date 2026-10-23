package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.FF_SCALAR;
import static frc.robot.subsystems.shooter.ShooterConstants.KA;
import static frc.robot.subsystems.shooter.ShooterConstants.KS;
import static frc.robot.subsystems.shooter.ShooterConstants.KV;
import static frc.robot.subsystems.shooter.ShooterConstants.TOLERANCE;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import frc.robot.RobotState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

public class Shooter implements Mechanism {

  private final ShooterIO io;
  // Feedforward: handles steady-state voltage (V = KS + KV * velocity)
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV, KA);
  // PID: handles error correction on top of feedforward
  // P only — no I term (causes integral windup oscillation)
  private final PIDController pid = new PIDController(0.001, 0.1, 0.1, 0.02);
  // Slew rate limiter: ramps the target velocity gradually (rad/s per second)
  // This prevents current spikes that cause oscillation with a 20A limit
  private final SlewRateLimiter targetRamper = new SlewRateLimiter(800);
  public boolean controllerEnabled = true;
  private AngularVelocity targetSpeed = RadiansPerSecond.of(0);
  private double rampedTarget = 0.0;
  private double feedForwardScalar;

  /**
   * Initializes the shooter with a Shooter IO
   *
   * @param io A ShooterIO implementing instance
   */
  public Shooter(ShooterIO io) {
    feedForwardScalar = FF_SCALAR;
    this.io = io;
  }

  @Override
  public void periodic() {

    if (SmartDashboard.getNumber("Shooter/FeedForward_Scalar", feedForwardScalar)
        != feedForwardScalar
        || SmartDashboard.getNumber(
        "Shooter/KS",
        feedforward.getKs()
            * SmartDashboard.getNumber("Shooter/FeedForward_Scalar", feedForwardScalar))
        * SmartDashboard.getNumber("Shooter/FeedForward_Scalar", feedForwardScalar)
        != feedforward.getKs()
        || SmartDashboard.getNumber("Shooter/KA", feedforward.getKa()) != feedforward.getKa()
        || SmartDashboard.getNumber(
        "Shooter/KV",
        feedforward.getKv()
            * SmartDashboard.getNumber("Shooter/FeedForward_Scalar", feedForwardScalar))
        * SmartDashboard.getNumber("Shooter/FeedForward_Scalar", feedForwardScalar)
        != feedforward.getKv()) {
      feedForwardScalar = SmartDashboard.getNumber("Shooter/FeedForward_Scalar", feedForwardScalar);
      feedforward.setKa(SmartDashboard.getNumber("Shooter/KA", feedforward.getKa()));
      feedforward.setKv(
          SmartDashboard.getNumber(
              "Shooter/KV",
              feedforward.getKv()
                  * SmartDashboard.getNumber("Shooter/FeedForward_Scalar", feedForwardScalar))
              * SmartDashboard.getNumber("Shooter/FeedForward_Scalar", feedForwardScalar));
      feedforward.setKa(
          SmartDashboard.getNumber(
              "Shooter/KS",
              feedforward.getKs()
                  * SmartDashboard.getNumber("Shooter/FeedForward_Scalar", feedForwardScalar))
              * SmartDashboard.getNumber("Shooter/FeedForward_Scalar", feedForwardScalar));
    }
    io.updateInputs(inputs);
    RobotState.getInstance().shooterAtSpeed =
        isAtSetpoint() && targetSpeed.gt(RadiansPerSecond.of(0));
    if (controllerEnabled) {
      // Ramp toward the target to avoid current spikes
      rampedTarget = targetRamper.calculate(targetSpeed.in(RadiansPerSecond));

      double ff = feedforward.calculate(rampedTarget);
      double pidOutput = pid.calculate(inputs.shooterWheelVelocityRadPerSec, rampedTarget);
      double voltage = ff;

      // Clamp to valid voltage range
      voltage =
          Math.max(-ShooterConstants.MAX_VOLTAGE, Math.min(ShooterConstants.MAX_VOLTAGE, voltage));
      io.setVoltage(voltage);

      Logger.recordOutput("Shooter/FFVoltage", ff);
      Logger.recordOutput("Shooter/PIDVoltage", pidOutput);
      Logger.recordOutput("Shooter/CommandedVoltage", voltage);
      Logger.recordOutput("Shooter/RampedTargetRadPerSec", rampedTarget);
      Logger.recordOutput("Shooter/Setpoint", targetSpeed.in(RadiansPerSecond));
    }

    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/AtSetpoint", isAtSetpoint());
    RobotState.getInstance().shooterAtSpeed = isAtSetpoint();
  }

  /**
   * Stop the shooter
   *
   * @return A command that stops the shooter
   */
  public Command stop() {
    return Commands.runOnce(
            () -> {
              controllerEnabled = false;
              targetSpeed = RadiansPerSecond.of(0);
              rampedTarget = 0.0;
              targetRamper.reset(0.0);
              io.stop();
            },
            this)
        .withName("shooter_stop_please");
  }

  /**
   * Sets raw voltage for the shooter
   *
   * @param volts The volts amount to set it to (max 12)
   * @return A Command that sets the shooter voltage
   */
  public Command setVoltage(double volts) {
    return Commands.runOnce(() -> controllerEnabled = false, this)
        .andThen(Commands.run(() -> io.setVoltage(volts), this));
  }

  /**
   * Set the target velocity of the shooter
   *
   * @param velocity A supplier that returns the target velocity of the shooter
   * @return A command that sets the target velocity of the shooter to the value returned by the
   * supplier
   */
  public Command setTargetVelocity(Supplier<AngularVelocity> velocity) {
    return Commands.run(
            () -> {
              targetSpeed = velocity.get();
              controllerEnabled = true;
            },
            this)
        .finallyDo(io::stop)
        .withName("setTargetVelocity");
  }

  /**
   * Set the target velocity of the shooter
   *
   * @param velocity The target velocity of the shooter
   * @return A command that sets the target velocity of the shooter to the specified value
   */
  public Command setTargetVelocity(AngularVelocity velocity) {
    return Commands.run(
            () -> {
              targetSpeed = velocity;
              controllerEnabled = true;
            },
            this)
        .withName("setTargetVelocity");
  }

  public Command setTargetVelocityRepeatedly(AngularVelocity velocity) {
    return Commands.repeatingSequence(
            Commands.runOnce(
                () -> {
                  targetSpeed = velocity;
                  controllerEnabled = true;
                },
                this),
            Commands.waitSeconds(0.02))
        .withName("setTargetVelocityRepeatedly");
  }

  /**
   * Check if the shooter is at setpoint (+- TOLERANCE)
   *
   * @return A boolean asserting whether the shooter is at setpoint (true for yes)
   */
  public boolean isAtSetpoint() {
    return Math.abs(inputs.shooterWheelVelocityRadPerSec - (targetSpeed.in(RadiansPerSecond)))
        < TOLERANCE;
  }

  // TODO: empirically determine the relationship between distance and air time
  public Time getAirTimeSeconds(Distance distance) {
    return Seconds.of(0.0617 * distance.in(Meters) + 0.872);
  }

  // TODO: empirically determine the relationship between distance and shooter velocity

  /**
   * Calculates a setpoint (in radians per second) based on distance
   *
   * @param distance Distance
   * @return Setpoint in radians per second
   */
  public Supplier<AngularVelocity> calculateSetpoint(Supplier<Distance> distance) {
    // calculate rpm depending on distance
    return () -> {
      AngularVelocity velocity =
          RPM.of(
              25.30184 * Math.pow(distance.get().in(Meters), 2)
                  - 47.12642 * distance.get().in(Meters)
                  + 1001.70713);
      if (velocity.gt(RPM.of(1550))) {
        return RPM.of(1550);
      } else {
        return velocity;
      }
    };
  }

  /**
   * Get the current target velocity of the shooter
   *
   * @return The target velocity of the shooter
   */
  public AngularVelocity getSetpoint() {
    return targetSpeed;
  }
}
