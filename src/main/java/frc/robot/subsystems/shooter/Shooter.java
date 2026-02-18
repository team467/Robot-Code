package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.TOLERANCE;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
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

  private final LinearSystem<N1, N1, N1> shooterWheel =
      LinearSystemId.identifyVelocitySystem(ShooterConstants.KV, ShooterConstants.KA);

  private final LinearQuadraticRegulator<N1, N1, N1> controller =
      new LinearQuadraticRegulator<>(
          shooterWheel,
          VecBuilder.fill(80), // Velocity error tolerance
          VecBuilder.fill(2), // Control effort (voltage)
          //          VecBuilder.fill(60), // Velocity error tolerance
          //          VecBuilder.fill(2.50), // Control effort (voltage) tolerance
          0.020);

  private final KalmanFilter<N1, N1, N1> observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          shooterWheel,
          VecBuilder.fill(3.0), // How accurate model is
          VecBuilder.fill(0.01), // How accurate encoder data is
          0.020);

  private final LinearSystemLoop<N1, N1, N1> loop =
      new LinearSystemLoop<>(
          shooterWheel, controller, observer, ShooterConstants.MAX_VOLTAGE, 0.020);

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
      loop.setNextR(VecBuilder.fill(targetRadPerSec));
      loop.correct(VecBuilder.fill(inputs.shooterWheelVelocityRadPerSec));
      loop.predict(0.020);
      io.setVoltage(loop.getU(0));
    }

    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput(
        "Shooter/ShooterWheelRPM", inputs.shooterWheelVelocityRadPerSec * 60.0 / (2.0 * Math.PI));
    Logger.recordOutput("Shooter/TargetRPM", targetRadPerSec * 60.0 / (2.0 * Math.PI));
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
