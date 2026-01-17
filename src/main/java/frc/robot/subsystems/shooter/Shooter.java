package frc.robot.subsystems.shooter;

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

  public void setPercent(double percent) {
    io.setPercent(percent);
    setpointEnabled = false;
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
    setpointEnabled = false;
  }

  public void stop() {
    io.stop();
    setpointEnabled = false;
  }

  public void setTargetVelocity(double setpoint) {
    io.setTargetVelocity(setpoint);
    setpointEnabled = true;
  }

  
}
