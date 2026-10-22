package first.robot.mechanisms.Intake.Extend;

public class IntakeExtendIOSim implements IntakeExtendIO {
  private static final double DT = 0.02;
  private static final double MAX_SPEED = 2.0;
  private static final double VOLTS_TO_SPEED = MAX_SPEED / 12.0;
  private static final double LIMIT_POSITION = 0.0;

  private double position = 0.0;
  private double velocity = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeExtendIOInputs inputs) {
    inputs.connected = true;
    inputs.position = position;
    inputs.velocity = velocity;
    inputs.appliedVolts = appliedVolts;
    inputs.outputCurrentAmps = Math.abs(appliedVolts) * 2.0;
    inputs.temperatureCelsius = 25.0;
    inputs.limitSwitchPressed = position <= LIMIT_POSITION;
  }

  @Override
  public void resetPosition() {
    position = 0.0;
  }

  @Override
  public void applyOutputs(IntakeExtendIOOutputs outputs) {
    switch (outputs.mode) {
      case POSITION -> {
        double error = outputs.targetPosition - position;
        double step = Math.max(-MAX_SPEED * DT, Math.min(MAX_SPEED * DT, error));
        velocity = step / DT;
        appliedVolts = velocity / VOLTS_TO_SPEED;
      }
      case VOLTAGE -> {
        appliedVolts = outputs.targetVolts;
        velocity = appliedVolts * VOLTS_TO_SPEED;
      }
      case STOP -> {
        appliedVolts = 0.0;
        velocity = 0.0;
      }
    }
    position += velocity * DT;
  }
}
