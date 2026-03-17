package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private final DCMotorSim motorSim;
  private double appliedVolts = 0.0;

  public ShooterIOSim() {
    // 3 NEO motors through a 2.5:1 gear ratio
    DCMotor gearbox = DCMotor.getNEO(3);
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.004, SHOOTER_WHEEL_GEAR_RATIO), gearbox);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    motorSim.update(0.02);

    double motorVelRadPerSec = motorSim.getAngularVelocityRadPerSec();
    double wheelVelRadPerSec = motorVelRadPerSec / SHOOTER_WHEEL_GEAR_RATIO;

    inputs.bottomMotorVelocityRadPerSec = motorVelRadPerSec;
    inputs.middleMotorVelocityRadPerSec = motorVelRadPerSec;
    inputs.topMotorVelocityRadPerSec = motorVelRadPerSec;

    inputs.bottomMotorAppliedVolts = appliedVolts;
    inputs.middleMotorAppliedVolts = appliedVolts;
    inputs.topMotorAppliedVolts = appliedVolts;

    inputs.bottomMotorCurrentAmps = motorSim.getCurrentDrawAmps() / 3.0;
    inputs.middleMotorCurrentAmps = motorSim.getCurrentDrawAmps() / 3.0;
    inputs.topMotorCurrentAmps = motorSim.getCurrentDrawAmps() / 3.0;
    inputs.totalAmps = motorSim.getCurrentDrawAmps();

    inputs.shooterWheelVelocityRadPerSec = wheelVelRadPerSec;
    inputs.shooterWheelPosition = motorSim.getAngularPositionRad() / SHOOTER_WHEEL_GEAR_RATIO;
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
