package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Indexer2022;
import org.apache.logging.log4j.Logger;

public class Indexer2022BackwardCMD extends CommandBase {
  private static final Logger LOGGER =
      RobotLogManager.getMainLogger(Indexer2022BackwardCMD.class.getName());
  private final Indexer2022 indexer;

  /**
   * Creates a new Indexer2022BackwardCMD.
   *
   * @param indexer The subsystem used by this command.
   */
  public Indexer2022BackwardCMD(Indexer2022 indexer) {
    this.indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LOGGER.debug("Setting indexer backwards");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.backward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
