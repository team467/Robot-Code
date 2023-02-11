package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ArmIOSim implements ArmIO {
  private final FlywheelSim extendSim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
  private final FlywheelSim rotateSim =
      new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);
  private double rotateRelativePosition = 0.0;
  private double extendAppliedVolts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    extendSim.update(0.02);
    rotateSim.update(0.02);

    double angleDiff = rotateSim.getAngularVelocityRadPerSec() * 0.02;
    rotateRelativePosition += angleDiff;

    inputs.extendPosition += extendSim.getAngularVelocityRadPerSec() * 0.02;
    inputs.extendVelocity = extendSim.getAngularVelocityRadPerSec();
    inputs.extendAppliedVolts = extendAppliedVolts;
    inputs.extendCurrent = Math.abs(extendSim.getCurrentDrawAmps());
    inputs.extendTemp = 0.0;

    inputs.rotatePosition = rotateRelativePosition;
    inputs.rotateVelocity = rotateSim.getAngularVelocityRadPerSec();
    inputs.rotateCurrent = Math.abs(rotateSim.getCurrentDrawAmps());
    inputs.rotateTemp = 0.0;
  }

  /**
   * It sets the voltage of the extendSim motor.
   *
   * @param volts The voltage to set the motor to.
   */
  @Override
  public void setExtendVoltage(double volts) {
    extendSim.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
  }

  /**
   * This function sets the voltage of the motor to the value of the parameter volts
   *
   * @param volts The voltage to set the motor to.
   */
  @Override
  public void setRotateVoltage(double volts) {
    rotateSim.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
  }
}
