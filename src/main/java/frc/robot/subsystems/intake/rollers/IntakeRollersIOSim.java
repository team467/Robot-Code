package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.MathUtil;

public class IntakeRollersIOSim implements IntakeRollersIO {
  private double intakeVolts = 0.0;
  private double intakePercent = 0.0;

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    inputs.intakeVolts = intakeVolts;
    inputs.intakePercentOutput = intakePercent;
    inputs.intakeAmps = Math.abs(intakeVolts) * 2.0;
    inputs.intakeRPM = intakePercent * 6000.0;
  }

  @Override
  public void setPercentIntake(double intakePercent) {
    this.intakePercent = MathUtil.clamp(intakePercent, -1.0, 1.0);
    this.intakeVolts = this.intakePercent * 12.0;
  }

  @Override
  public void setVoltageIntake(double intakeVolts) {
    this.intakeVolts = MathUtil.clamp(intakeVolts, -12.0, 12.0);
    this.intakePercent = this.intakeVolts / 12.0;
  }

  @Override
  public void stop() {
    this.intakeVolts = 0.0;
    this.intakePercent = 0.0;
  }
}
