package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.TOLERANCE;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
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
  private final PIDController pid = new PIDController(0.02, 0.0, 0.0);

  // Slew rate limiter: ramps the target velocity gradually (rad/s per second)
  // This prevents current spikes that cause oscillation with a 20A limit
  private final SlewRateLimiter targetRamper = new SlewRateLimiter(100);

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

    if (controllerEnabled) {
      // Ramp toward the target to avoid current spikes
      rampedTarget = targetRamper.calculate(targetRadPerSec);

      double ff = feedforward.calculate(rampedTarget);
      double pidOutput = pid.calculate(inputs.shooterWheelVelocityRadPerSec, rampedTarget);
      double voltage = ff + pidOutput;

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
        "Shooter/ShooterWheelRPM", inputs.shooterWheelVelocityRadPerSec * 60.0 / (2.0 * Math.PI));
    Logger.recordOutput("Shooter/TargetRPM", targetRadPerSec * 60.0 / (2.0 * Math.PI));
    Logger.recordOutput("Shooter/RampedTargetRPM", rampedTarget * 60.0 / (2.0 * Math.PI));
    Logger.recordOutput(
        "Shooter/MiddleMotorRPM", inputs.middleMotorVelocityRadPerSec * 60.0 / (2.0 * Math.PI));
    Logger.recordOutput(
        "Shooter/BottomMotorRPM", inputs.bottomMotorVelocityRadPerSec * 60.0 / (2.0 * Math.PI));
    Logger.recordOutput(
        "Shooter/TopMotorRPM", inputs.topMotorVelocityRadPerSec * 60.0 / (2.0 * Math.PI));
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
        this);
  }

  public Command setVoltage(double volts) {
    return Commands.sequence(
        Commands.runOnce(() -> controllerEnabled = false, this),
        Commands.run(() -> io.setVoltage(volts), this));
  }

  public Command setTargetVelocityRPM(double rpm) {
    return Commands.runOnce(
        () -> {
          targetRadPerSec = ((rpm * 2 * Math.PI) / 60.0);
          controllerEnabled = true;
        },
        this);
  }

  public Command setTargetVelocityRadians(double radPerSec) {
    return Commands.runOnce(
        () -> {
          targetRadPerSec = radPerSec;
          controllerEnabled = true;
        },
        this);
  }

  public Command setTargetDistance(double distanceMeters) {
    return Commands.runOnce(
        () -> {
          targetRadPerSec = distanceMeters;
          controllerEnabled = true;
        });
  }

  public boolean isAtSetpoint() {
    return Math.abs(inputs.shooterWheelVelocityRadPerSec - (targetRadPerSec)) < TOLERANCE;
  }
}
