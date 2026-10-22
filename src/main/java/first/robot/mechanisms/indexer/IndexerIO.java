package first.robot.mechanisms.indexer;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean connected = false;
    public double feedUpPercentOutput = 0.0;
    public double feedUpVolts = 0.0;
    public double feedUpAmps = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public static enum IndexerIOOutputMode {
    VOLTAGE,
    STOP
  }

  public static class IndexerIOOutputs {
    @AutoLogOutput(key = "Indexer/Output Mode")
    public IndexerIOOutputMode mode = IndexerIOOutputMode.STOP;

    @AutoLogOutput(key = "Indexer/Target Volts")
    public double targetVolts = 0.0;
  }

  public default void applyOutputs(IndexerIOOutputs outputs) {}
}
