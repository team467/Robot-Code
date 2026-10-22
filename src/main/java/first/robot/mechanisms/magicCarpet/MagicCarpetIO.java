package first.robot.mechanisms.magicCarpet;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface MagicCarpetIO {
  @AutoLog
  public static class MagicCarpetIOInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double motorVelocity = 0.0;
  }

  public default void updateInputs(MagicCarpetIOInputs inputs) {}

  public static enum MagicCarpetIOOutputMode {
    DUTY_CYCLE,
    STOP
  }

  public static class MagicCarpetIOOutputs {
    @AutoLogOutput(key = "MagicCarpet/Output Mode")
    public MagicCarpetIOOutputMode mode = MagicCarpetIOOutputMode.STOP;

    @AutoLogOutput(key = "MagicCarpet/Target Speed")
    public double targetSpeed = 0.0;
  }

  public default void applyOutputs(MagicCarpetIOOutputs outputs) {}
}
