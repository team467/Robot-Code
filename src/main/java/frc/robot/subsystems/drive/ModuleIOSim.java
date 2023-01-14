package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ModuleIOSim implements ModuleIO {
  private final FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
  private final FlywheelSim turnSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);
  private double turnRelativePosition = 0.0;
  private double turnAbsolutePosition = Math.random() * 2.0 * Math.PI;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(0.02);
    turnSim.update(0.02);

    double angleDiff = turnSim.getAngularVelocityRadPerSec() * 0.02;

    turnRelativePosition += angleDiff;
    turnAbsolutePosition += angleDiff;
    while (turnAbsolutePosition < 0) {
      turnAbsolutePosition += 2.0 * Math.PI;
    }
    while (turnAbsolutePosition > 2.0 * Math.PI) {
      turnAbsolutePosition -= 2.0 * Math.PI;
    }

    inputs.drivePosition += driveSim.getAngularVelocityRadPerSec() * 0.02;
    inputs.driveVelocity = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrent = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};
    inputs.driveTemp = new double[] {};

    inputs.turnPositionAbsolute = turnAbsolutePosition;
    inputs.turnPosition = turnRelativePosition;
    inputs.turnVelocity = turnSim.getAngularVelocityRadPerSec();
    inputs.turnCurrent = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};
    inputs.turnTemp = new double[] {};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDriveBrakeMode(boolean brake) {}

  @Override
  public void setTurnBrakeMode(boolean brake) {}
}
