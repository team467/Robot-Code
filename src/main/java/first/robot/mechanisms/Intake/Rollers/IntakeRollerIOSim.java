package first.robot.mechanisms.Intake.Rollers;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private static final double DT = 0.02;
  private static final double FREE_SPEED = 100.0;
  private static final double TIME_CONSTANT = 0.1;

  private double position = 0.0;
  private double velocity = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.connected = true;
    inputs.position = position;
    inputs.velocity = velocity;
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(appliedVolts);
    inputs.statorCurrentAmps = Math.abs(appliedVolts) * 2.0;
    inputs.temperatureCelsius = 25.0;
  }

  @Override
  public void applyOutputs(IntakeRollerIOOutputs outputs) {
    appliedVolts = outputs.mode == RollerIOOutputMode.VOLTAGE ? outputs.targetVolts : 0.0;
    double target = appliedVolts / 12.0 * FREE_SPEED;
    velocity += (target - velocity) * (DT / TIME_CONSTANT);
    position += velocity * DT;
  }
}
