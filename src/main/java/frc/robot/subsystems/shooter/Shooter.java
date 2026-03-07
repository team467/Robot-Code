package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.TOLERANCE;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.RobotState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public boolean controllerEnabled = true;
  private double targetRadPerSec = 0.0;
  private double rampedTarget = 0.0;

  // Feedforward: handles steady-state voltage (V = KS + KV * velocity)
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(ShooterConstants.KS, ShooterConstants.KV);
  // PID: handles error correction on top of feedforward
  // P only — no I term (causes integral windup oscillation)
  private final PIDController pid = new PIDController(0.001, 0.1, 0.1, 0.02);

  // Slew rate limiter: ramps the target velocity gradually (rad/s per second)
  // This prevents current spikes that cause oscillation with a 20A limit
  private final SlewRateLimiter targetRamper = new SlewRateLimiter(800);

  public Shooter(ShooterIO io) {
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
    io.updateInputs(inputs);
    RobotState.getInstance().shooterAtSpeed = isAtSetpoint();
    if (controllerEnabled) {
      // Ramp toward the target to avoid current spikes
      rampedTarget = targetRamper.calculate(targetRadPerSec);

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
    }

    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput(
        "Shooter/ShooterWheelRPM",
        Units.radiansPerSecondToRotationsPerMinute(inputs.shooterWheelVelocityRadPerSec));
    Logger.recordOutput(
        "Shooter/TargetRPM", Units.radiansPerSecondToRotationsPerMinute(targetRadPerSec));
    Logger.recordOutput("Shooter/RampedTargetRPM", rampedTarget);
    Logger.recordOutput(
        "Shooter/MiddleMotorRPM",
        Units.radiansPerSecondToRotationsPerMinute(inputs.middleMotorVelocityRadPerSec));
    Logger.recordOutput(
        "Shooter/BottomMotorRPM",
        Units.radiansPerSecondToRotationsPerMinute(inputs.bottomMotorVelocityRadPerSec));
    Logger.recordOutput(
        "Shooter/TopMotorRPM",
        Units.radiansPerSecondToRotationsPerMinute(inputs.topMotorVelocityRadPerSec));
    Logger.recordOutput("Shooter/MiddleMotorAmps", inputs.middleMotorCurrentAmps);
    Logger.recordOutput("Shooter/BottomMotorAmps", inputs.bottomMotorCurrentAmps);
    Logger.recordOutput("Shooter/TopMotorAmps", inputs.topMotorCurrentAmps);
    Logger.recordOutput("Shooter/TotalAmps", inputs.totalAmps);
    Logger.recordOutput("Shooter/AtSetpoint", isAtSetpoint());
  }

  public void runCharacterization(double voltage) {
    io.setVoltage(voltage);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(() -> controllerEnabled = false, this)
        .andThen(run(() -> runCharacterization(0.0)).withTimeout(1.0))
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(() -> controllerEnabled = false, this)
        .andThen(run(() -> runCharacterization(0.0)).withTimeout(1.0))
        .andThen(sysId.dynamic(direction));
  }

  public Command stop() {
    return Commands.runOnce(
            () -> {
              controllerEnabled = false;
              targetRadPerSec = 0.0;
              rampedTarget = 0.0;
              targetRamper.reset(0.0);
              io.stop();
            },
            this)
        .withName("shooter_stop_please");
  }

  public Command setVoltage(double volts) {
    return Commands.sequence(
        Commands.runOnce(() -> controllerEnabled = false, this),
        Commands.run(() -> io.setVoltage(volts), this));
  }

  public Command setTargetVelocityRPM(double rpm) {
    return Commands.run(
            () -> {
              targetRadPerSec = ((rpm / 2 * Math.PI) * 60.0);
              controllerEnabled = true;
            },
            this)
        .withName("setTargetVelocityRPM");
  }

  public Command setTargetVelocityRadians(double radPerSec) {
    return Commands.run(
            () -> {
              targetRadPerSec = radPerSec;
              controllerEnabled = true;
            },
            this)
        .withName("setTargetVelocityRadians");
  }

  public boolean isAtSetpoint() {
    return Math.abs(inputs.shooterWheelVelocityRadPerSec - (targetRadPerSec)) < TOLERANCE;
  }

  // TODO: empirically determine the relationship between distance and air time
  public double getAirTimeSeconds(DoubleSupplier distance) {
    return distance.getAsDouble();
  }

  // TODO: empirically determine the relationship between distance and shooter velocity

  public double calculateSetpoint(DoubleSupplier distance) {
    // calculate radians per second depending on distance
    return 0.0;
  }

  public double getSetpoint() {
    return inputs.setpointRPM;
  }
}
