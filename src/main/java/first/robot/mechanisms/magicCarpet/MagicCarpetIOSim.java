package first.robot.mechanisms.magicCarpet;

public class MagicCarpetIOSim implements MagicCarpetIO {
  private static final double DT = 0.02;
  private static final double FREE_SPEED_RPM = 5676.0;
  private static final double TIME_CONSTANT = 0.1;

  private double velocityRpm = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(MagicCarpetIOInputs inputs) {
    inputs.connected = true;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(appliedVolts) * 1.5;
    inputs.motorVelocity = velocityRpm;
  }

  @Override
  public void applyOutputs(MagicCarpetIOOutputs outputs) {
    double speed =
        outputs.mode == MagicCarpetIOOutputMode.DUTY_CYCLE
            ? Math.clamp(outputs.targetSpeed, 0.0, 1.0)
            : 0.0;
    appliedVolts = speed * 12.0;
    velocityRpm += (speed * FREE_SPEED_RPM - velocityRpm) * (DT / TIME_CONSTANT);
  }
}
