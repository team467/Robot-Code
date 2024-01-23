package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private DCMotorSim shooterLeaderSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
  private DCMotorSim shooterFollowerSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
  private DCMotorSim indexerSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterLeaderVelocityRadPerSec = shooterLeaderSim.getAngularVelocityRadPerSec();
    inputs.shooterFollowerVelocityRadPerSec = shooterFollowerSim.getAngularVelocityRadPerSec();
    inputs.indexerVelocityRadPerSec = indexerSim.getAngularVelocityRadPerSec();
  }

  public void setShooterVeltage(double volts) {
    shooterLeaderSim.setInputVoltage(volts);
    shooterFollowerSim.setInputVoltage(-volts);
  }

  public void setIndexerVoltage(double volts) {
    indexerSim.setInputVoltage(volts);
  }
}
