package first.robot.mechanisms.shooter;

import static first.robot.mechanisms.shooter.ShooterConstants.*;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private static final double DT = 0.02;

  private final FlywheelSim sim =
      new FlywheelSim(Models.flywheelFromSysId(KV, KA), DCMotor.getNEO(4));
  private double wheelPositionRotations = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double wheelVelocity = sim.getAngularVelocity();
    double motorVelocity = wheelVelocity * SHOOTER_WHEEL_GEAR_RATIO;
    double motorCurrent = Math.abs(sim.getCurrentDraw()) / 4.0;

    inputs.connected = true;
    inputs.topLeftMotorVelocityRadPerSec = motorVelocity;
    inputs.topRightMotorVelocityRadPerSec = motorVelocity;
    inputs.bottomLeftMotorVelocityRadPerSec = motorVelocity;
    inputs.bottomRightMotorVelocityRadPerSec = motorVelocity;
    inputs.topLeftMotorAppliedVolts = appliedVolts;
    inputs.topRightMotorAppliedVolts = appliedVolts;
    inputs.bottomLeftMotorAppliedVolts = appliedVolts;
    inputs.bottomRightMotorAppliedVolts = appliedVolts;
    inputs.topLeftMotorCurrentAmps = motorCurrent;
    inputs.topRightMotorCurrentAmps = motorCurrent;
    inputs.bottomLeftMotorCurrentAmps = motorCurrent;
    inputs.bottomRightMotorCurrentAmps = motorCurrent;
    inputs.totalAmps = motorCurrent * 4.0;
    inputs.busVoltage = 12.0;
    inputs.shooterWheelVelocityRadPerSec = wheelVelocity;
    inputs.shooterWheelPosition = wheelPositionRotations;
  }

  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    appliedVolts =
        outputs.mode == ShooterIOOutputMode.VOLTAGE
            ? Math.clamp(outputs.targetVolts, -MAX_VOLTAGE, MAX_VOLTAGE)
            : 0.0;
    sim.setInputVoltage(appliedVolts);
    sim.update(DT);
    wheelPositionRotations += sim.getAngularVelocity() * DT / (2 * Math.PI);
  }
}
