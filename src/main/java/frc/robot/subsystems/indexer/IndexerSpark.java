package frc.robot.subsystems.indexer;

import frc.lib.mechanisms.flywheels.SparkFlywheel;
import org.wpilib.command3.Command;

public class IndexerSpark implements Indexer {

  SparkFlywheel flywheel;

  @Override
  public Command feed() {
    return flywheel.run();
  }

  @Override
  public Command reverse() {
    return flywheel.reverse();
  }

  @Override
  public Command stop() {
    return flywheel.stop();
  }
}
