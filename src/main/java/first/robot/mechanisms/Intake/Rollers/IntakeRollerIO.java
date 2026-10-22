package first.robot.mechanisms.Intake.Rollers;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public boolean connected = true;

    public double position = 0.0;
    public double velocity = 0.0;

    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {}

  public static enum RollerIOOutputMode {
    VOLTAGE,
    STOP
  }

  public static class IntakeRollerIOOutputs {
    @AutoLogOutput(key = "IntakeRoller/Output Mode")
    public RollerIOOutputMode mode = RollerIOOutputMode.STOP;

    @AutoLogOutput(key = "IntakeRoller/Target Volts")
    public double targetVolts = 0.0;
  }

  public default void applyOutputs(IntakeRollerIOOutputs outputs) {}
}
