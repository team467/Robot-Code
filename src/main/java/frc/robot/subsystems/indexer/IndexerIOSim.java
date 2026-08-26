package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import frc.robot.sim.BallSimulator;

public class IndexerIOSim implements IndexerIO {
  private double feedUpVolts = 0.0;
  private double feedUpPercent = 0.0;
  private boolean manualLeftSwitch = false;
  private boolean manualRightSwitch = false;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.feedUpVolts = feedUpVolts;
    inputs.feedUpPercentOutput = feedUpPercent;
    inputs.feedUpAmps = Math.abs(feedUpVolts) * 1.5;

    boolean hasBalls =
        BallSimulator.getInstance().hasBalls() || manualLeftSwitch || manualRightSwitch;
    inputs.ballAtLeftSwitch = hasBalls;
    inputs.ballAtRightSwitch = hasBalls;
  }

  @Override
  public void setPercent(double indexPercent, double feedUpPercent) {
    this.feedUpPercent = MathUtil.clamp(feedUpPercent, -1.0, 1.0);
    this.feedUpVolts = this.feedUpPercent * 12.0;
  }

  @Override
  public void setVoltage(double feedUpVolts) {
    this.feedUpVolts = MathUtil.clamp(feedUpVolts, -12.0, 12.0);
    this.feedUpPercent = this.feedUpVolts / 12.0;
  }

  @Override
  public void stop() {
    feedUpVolts = 0.0;
    feedUpPercent = 0.0;
  }

  @Override
  public boolean isLeftSwitchPressed() {
    return BallSimulator.getInstance().hasBalls() || manualLeftSwitch;
  }

  @Override
  public boolean isRightSwitchPressed() {
    return BallSimulator.getInstance().hasBalls() || manualRightSwitch;
  }
}
