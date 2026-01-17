package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  public boolean setpointEnabled = false;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (setpointEnabled) {
      io.goToSetpoint();
    }

    Logger.processInputs("Shooter", inputs);
  }

  public Command stop() {
    return Commands.runOnce(
        () -> {
          io.stop();
        },
        this);
  }

  public Command setPercent(double percent) {
    return Commands.run(
        () -> {
          io.setPercent(percent);
        },
        this);
  }

  public Command setVoltage(double volts) {
    return Commands.run(
        () -> {
          io.setVoltage(volts);
        },
        this);
  }

  public void setTargetVelocity(double setpoint) {
    io.setTargetVelocity(setpoint);
    setpointEnabled = true;
  }
}
