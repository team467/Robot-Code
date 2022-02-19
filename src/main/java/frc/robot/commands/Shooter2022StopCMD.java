package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Indexer2022;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Shooter2022;
import frc.robot.subsystems.Spitter2022;
import org.apache.logging.log4j.Logger;

public class Shooter2022StopCMD extends CommandBase {

  private static final Logger LOGGER =
      RobotLogManager.getMainLogger(Shooter2022StopCMD.class.getName());

  private final Command llamaNeckStop;
  private final Command spitterStop;
  private final Command indexerStop;
  
  private final LlamaNeck2022 llamaNeck;
  private final Indexer2022 indexer;
  private final Spitter2022 spitter;

  public Shooter2022StopCMD(Shooter2022 shooter) {
    super();
    this.llamaNeck = shooter.llamaNeck2022;
    this.spitter = shooter.spitter2022;
    this.indexer = shooter.indexer2022;

    this.llamaNeckStop = new LlamaNeck2022StopCMD(llamaNeck);
    this.spitterStop = new Spitter2022StopCMD(spitter);
    this.indexerStop = new Indexer2022StopCMD(indexer);



    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    LOGGER.debug("Stopping shooter system.");
  }

  @Override
  public void execute() {
    llamaNeckStop.schedule();
    spitterStop.schedule();
    indexerStop.schedule();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
