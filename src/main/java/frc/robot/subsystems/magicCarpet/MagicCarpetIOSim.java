package frc.robot.subsystems.magicCarpet;

import edu.wpi.first.math.MathUtil;

public class MagicCarpetIOSim implements MagicCarpetIO {
  private double speed = 0.0;

  @Override
  public void updateInputs(MagicCarpetIOInputs inputs) {
    inputs.appliedVolts = speed * 12.0;
    inputs.motorVelocity = speed * 5676.0;
    inputs.currentAmps = Math.abs(speed) * 8.0;
  }

  @Override
  public void setSpeed(double speed) {
    this.speed = MathUtil.clamp(speed, 0.0, 1.0);
  }

  @Override
  public void stop() {
    this.speed = 0.0;
  }
}
