package frc.robot.subsystems.shooter;

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
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final LinearSystem<N1, N1, N1> flywheel =
      LinearSystemId.identifyVelocitySystem(ShooterConstants.KV, ShooterConstants.KA);

  private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          flywheel,
          VecBuilder.fill(ShooterConstants.VELOCITY_TOLERANCE),
          VecBuilder.fill(ShooterConstants.MAX_VOLTAGE),
          0.020);

  private final KalmanFilter<N1, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          flywheel,
          VecBuilder.fill(3.0), // How accurate model is
          VecBuilder.fill(0.01), // How accurate encoder data is
          0.020);

  private final LinearSystemLoop<N1, N1, N1> m_loop =
      new LinearSystemLoop<>(
          flywheel, m_controller, m_observer, ShooterConstants.MAX_VOLTAGE, 0.020);

  public boolean setpointEnabled = false;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    //    if (setpointEnabled) {
    //      io.goToSetpoint();
    //    }

    if (setpointEnabled) {
      m_loop.setNextR(VecBuilder.fill(inputs.setpointRPM * (Math.PI / 30.0)));
      m_loop.correct(VecBuilder.fill(inputs.shooterLeaderVelocityRadPerSec));
      m_loop.predict(0.020);
      io.setVoltage(m_loop.getU(0));
    }

    Logger.processInputs("Shooter", inputs);
  }

  public Command stop() {
    return Commands.runOnce(
        () -> {
          io.stop();
          setpointEnabled = false;
        },
        this);
  }

  public Command setPercent(double percent) {
    return Commands.sequence(
        Commands.runOnce(() -> setpointEnabled = false, this),
        Commands.run(() -> io.setVoltage(percent), this));
  }

  public Command setVoltage(double volts) {
    return Commands.sequence(
        Commands.runOnce(() -> setpointEnabled = false, this),
        Commands.run(() -> io.setVoltage(volts), this));
  }

  public Command setTargetVelocity(double setpoint) {
    return Commands.sequence(
        Commands.runOnce(() -> setpointEnabled = true, this),
        Commands.runEnd(() -> io.setTargetVelocity(setpoint), () -> setpointEnabled = false, this));
  }
}
