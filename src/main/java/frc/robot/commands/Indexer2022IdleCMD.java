package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Indexer2022;
import org.apache.logging.log4j.Logger;

public class Indexer2022IdleCMD extends CommandBase {
  private static final Logger LOGGER =
      RobotLogManager.getMainLogger(Indexer2022IdleCMD.class.getName());
  private final Indexer2022 indexer;

  public Indexer2022IdleCMD(Indexer2022 indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    LOGGER.debug("Setting indexer Slow Forward");
  }

  @Override
  public void execute() {
    indexer.idle();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
