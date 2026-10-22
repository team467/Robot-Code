package first.robot.mechanisms.indexer;

public class IndexerIOSim implements IndexerIO {
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.connected = true;
    inputs.feedUpPercentOutput = appliedVolts / 12.0;
    inputs.feedUpVolts = appliedVolts;
    inputs.feedUpAmps = Math.abs(appliedVolts) * 1.5;
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {
    appliedVolts =
        outputs.mode == IndexerIOOutputMode.VOLTAGE
            ? Math.clamp(outputs.targetVolts, -12.0, 12.0)
            : 0.0;
  }
}
