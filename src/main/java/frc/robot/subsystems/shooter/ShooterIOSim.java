package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim flywheelSim;
  private double appliedVolts = 0.0;
  private double wheelPositionRad = 0.0;

  public ShooterIOSim() {
    // 3 NEOs driving the flywheel mechanism through the gear ratio
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(3), 0.003, SHOOTER_WHEEL_GEAR_RATIO),
            DCMotor.getNEO(3));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    flywheelSim.setInputVoltage(MathUtil.clamp(appliedVolts, -MAX_VOLTAGE, MAX_VOLTAGE));
    flywheelSim.update(0.02);

    double wheelRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
    wheelPositionRad += wheelRadPerSec * 0.02;

    double motorRadPerSec = wheelRadPerSec * SHOOTER_WHEEL_GEAR_RATIO;
    double currentAmpsPerMotor = Math.abs(flywheelSim.getCurrentDrawAmps()) / 3.0;

    inputs.shooterWheelVelocityRadPerSec = wheelRadPerSec;
    inputs.shooterWheelPosition = wheelPositionRad;

    inputs.bottomMotorVelocityRadPerSec = motorRadPerSec;
    inputs.bottomMotorAppliedVolts = appliedVolts;
    inputs.bottomMotorCurrentAmps = currentAmpsPerMotor;

    inputs.middleMotorVelocityRadPerSec = motorRadPerSec;
    inputs.middleMotorAppliedVolts = appliedVolts;
    inputs.middleMotorCurrentAmps = currentAmpsPerMotor;

    inputs.topMotorVelocityRadPerSec = motorRadPerSec;
    inputs.topMotorAppliedVolts = appliedVolts;
    inputs.topMotorCurrentAmps = currentAmpsPerMotor;

    inputs.totalAmps = flywheelSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}
