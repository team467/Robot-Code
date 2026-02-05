package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

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

  public boolean controllerEnabled = false;
  private double targetRadPerSec = 0.0;

  private final LinearSystem<N1, N1, N1> flywheel =
      LinearSystemId.identifyVelocitySystem(ShooterConstants.KV, ShooterConstants.KA);

  private final LinearQuadraticRegulator<N1, N1, N1> controller =
      new LinearQuadraticRegulator<>(
          flywheel,
          VecBuilder.fill(8.0), // Velocity error tolerance
          VecBuilder.fill(12.0), // Control effort (voltage) tolerance
          0.020);

  private final KalmanFilter<N1, N1, N1> observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          flywheel,
          VecBuilder.fill(3.0), // How accurate model is
          VecBuilder.fill(0.01), // How accurate encoder data is
          0.020);

  private final LinearSystemLoop<N1, N1, N1> loop =
      new LinearSystemLoop<>(flywheel, controller, observer, ShooterConstants.MAX_VOLTAGE, 0.020);

  public Shooter(ShooterIO io) {
    this.io = io;

    sysId =
        new SysIdRoutine(
            new Config(
                null,
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
      loop.correct(VecBuilder.fill(inputs.shooterLeaderVelocityRadPerSec));
      loop.predict(0.020);
      io.setVoltage(loop.getU(0));
    }

    Logger.processInputs("Shooter", inputs);
  }

  public void runCharacterization(double voltage) {
    io.setVoltage(voltage);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  public Command stop() {
    return Commands.runOnce(
        () -> {
          io.stop();
          controllerEnabled = false;
        },
        this);
  }

  public Command setPercent(double percent) {
    return Commands.sequence(
        Commands.runOnce(() -> controllerEnabled = false, this),
        Commands.run(() -> io.setVoltage(percent), this));
  }

  public Command setVoltage(double volts) {
    return Commands.sequence(
        Commands.runOnce(() -> controllerEnabled = false, this),
        Commands.run(() -> io.setVoltage(volts), this));
  }

  public Command setTargetVelocityRPM(double rpm) {
    return Commands.runOnce(
        () -> {
          targetRadPerSec = rpm * (2 * Math.PI / 60.0);
          controllerEnabled = true;
        },
        this);
  }
}
