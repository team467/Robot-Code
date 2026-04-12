package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.FF_SCALAR;
import static frc.robot.subsystems.shooter.ShooterConstants.KA;
import static frc.robot.subsystems.shooter.ShooterConstants.KS;
import static frc.robot.subsystems.shooter.ShooterConstants.KV;
import static frc.robot.subsystems.shooter.ShooterConstants.TOLERANCE;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.RobotState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public boolean controllerEnabled = true;
  private AngularVelocity targetSpeed = RadiansPerSecond.of(0);
  private double rampedTarget = 0.0;

  // Feedforward: handles steady-state voltage (V = KS + KV * velocity)
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV, KA);
  // PID: handles error correction on top of feedforward
  // P only — no I term (causes integral windup oscillation)
  private final PIDController pid = new PIDController(0.001, 0.1, 0.1, 0.02);

  // Slew rate limiter: ramps the target velocity gradually (rad/s per second)
  // This prevents current spikes that cause oscillation with a 20A limit
  private final SlewRateLimiter targetRamper = new SlewRateLimiter(1600);
  private double feedForwardScalar;

  /**
   * Initializes the shooter with a Shooter IO
   *
   * @param io A ShooterIO implementing instance
   */
  public Shooter(ShooterIO io) {
    SmartDashboard.putNumber("Shooter/KS", KS * FF_SCALAR);
    SmartDashboard.putNumber("Shooter/KA", KA * FF_SCALAR);
    SmartDashboard.putNumber("Shooter/KV", KV * FF_SCALAR);
    SmartDashboard.putNumber("Shooter/FeedForward_Scalar", FF_SCALAR);
    feedForwardScalar = FF_SCALAR;
    this.io = io;
    sysId =
        new SysIdRoutine(
            new Config(
                Volts.per(Second).of(1),
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
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
   * Runs characterization voltage (used for FF)
   *
   * @param voltage The voltage to apply
   */
  public void runCharacterization(double voltage) {
    io.setVoltage(voltage);
  }

  /**
   * Runs SysID Quasistatic for feed forward
   *
   * @param direction The direction to run it in
   * @return A command that runs SysID Quasistatic
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(() -> controllerEnabled = false, this)
        .andThen(run(() -> runCharacterization(0.0)).withTimeout(1.0))
        .andThen(sysId.quasistatic(direction));
  }

  /**
   * Runs SysID Quasistatic for feed forward
   *
   * @param direction The direction to run it in
   * @return A command that runs SysID Dynamic
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(() -> controllerEnabled = false, this)
        .andThen(run(() -> runCharacterization(0.0)).withTimeout(1.0))
        .andThen(sysId.dynamic(direction));
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
   *     supplier
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
    // calculate rad/s depending on distance
    return () -> {
      AngularVelocity velocity = RadiansPerSecond.of(183.35 * distance.get().in(Meters) + 750.93);
      if (velocity.gt(RadiansPerSecond.of(5000))) {
        return RadiansPerSecond.of(5000);
      } else return velocity;
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
