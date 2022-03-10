package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Shooter2022;
import org.apache.logging.log4j.Logger;

public class Shooter2022FlushBallCMD extends CommandBase {

  private static final Logger LOGGER =
      RobotLogManager.getMainLogger(Shooter2022FlushBallCMD.class.getName());

  private final Command llamaNeckBackward;

  private final Command indexerBackward;

  private final Command spitterStop;

  public Shooter2022FlushBallCMD(Shooter2022 shooter) {
    super();

    this.llamaNeckBackward = new LlamaNeck2022BackwardCMD(shooter.llamaNeck2022);
    this.indexerBackward = new Indexer2022BackwardCMD(shooter.indexer2022);
    this.spitterStop = new Spitter2022StopCMD(shooter.spitter2022);

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    LOGGER.debug("Flushing system");
    llamaNeckBackward.schedule();
    indexerBackward.schedule();
    spitterStop.schedule();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
