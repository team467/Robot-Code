package first.robot.mechanisms.Intake.Extend;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface IntakeExtendIO {
  @AutoLog
  public static class IntakeExtendIOInputs {
    public boolean connected = true;

    public double position = 0.0;
    public double velocity = 0.0;

    public double appliedVolts = 0.0;
    public double outputCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;

    public boolean limitSwitchPressed = false;
  }

  public default void updateInputs(IntakeExtendIOInputs inputs) {}

  public static enum ExtendIOOutputMode {
    POSITION,
    VOLTAGE,
    STOP
  }

  public static class IntakeExtendIOOutputs {
    @AutoLogOutput(key = "IntakeExtend/Output Mode")
    public ExtendIOOutputMode mode = ExtendIOOutputMode.STOP;

    @AutoLogOutput(key = "IntakeExtend/Target Position")
    public double targetPosition = 0.0;

    @AutoLogOutput(key = "IntakeExtend/Target Voltage")
    public double targetVolts = 0.0;
  }

  public default void applyOutputs(IntakeExtendIOOutputs outputs) {}

  public default void resetPosition() {}
}
