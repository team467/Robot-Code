package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotConstants;

public class ModuleIOSim implements ModuleIO {
  private final FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
  private final FlywheelSim turnSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);
  private double turnRelativePosition = 0.0;
  private double turnAbsolutePosition = Math.random() * 2.0 * Math.PI;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private final double radsToMeters = 1 * (RobotConstants.get().moduleWheelDiameter() / 2);

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

    inputs.drivePositionMeters += driveSim.getAngularVelocityRadPerSec() * radsToMeters * 0.02;
    inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec() * radsToMeters;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};

    inputs.turnPositionAbsoluteRad = turnAbsolutePosition;
    inputs.turnPositionRad = turnRelativePosition;
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnCurrentAmps = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};
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
