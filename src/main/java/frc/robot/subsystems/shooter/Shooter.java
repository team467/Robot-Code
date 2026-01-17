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
    return Commands.sequence(
        Commands.runOnce(() -> setpointEnabled = false, this),
        Commands.run(() -> io.setVoltage(percent), this)
    );
  }

  public Command setVoltage(double volts) {
    return Commands.sequence(
        Commands.runOnce(() -> setpointEnabled = false, this),
        Commands.run(() -> io.setVoltage(volts), this)
    );
  }

  public Command setTargetVelocity(double setpoint) {
   return Commands.sequence(
       Commands.runOnce(() -> setpointEnabled = true, this),
       Commands.runEnd(
        () -> io.setTargetVelocity(setpoint),
        () -> setpointEnabled = false,
      this));
}


}
