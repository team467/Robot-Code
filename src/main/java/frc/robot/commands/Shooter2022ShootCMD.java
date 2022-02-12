// Turns everything to max and sends it to the spitter.
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer2022;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Shooter2022;
import frc.robot.subsystems.Spitter2022;

public class Shooter2022ShootCMD extends CommandBase {

  // private final double TIME_UNTIL_FINISHED = 0.1;
  private final double TIME_UNTIL_FINISHED = 2;

  private final LlamaNeck2022 llamaNeck;
  private final Spitter2022 spitter;

  private final Command llamaNeckStop;
  private final Command llamaNeckForward;

  private final Command indexerStop;
  private final Command indexerForward;

  private final Command spitterForward;

  private final Timer timer;

  public Shooter2022ShootCMD(
      Shooter2022 shooter, Indexer2022 indexer, LlamaNeck2022 llamaNeck, Spitter2022 spitter) {
    super();

    this.llamaNeck = llamaNeck;
    this.spitter = spitter;

    this.llamaNeckStop = new LlamaNeck2022StopCMD(llamaNeck);
    this.llamaNeckForward = new LlamaNeck2022ForwardCMD(llamaNeck);

    this.indexerStop = new Indexer2022StopCMD(indexer);
    this.indexerForward = new Indexer2022ForwardCMD(indexer);

    this.spitterForward = new Spitter2022ForwardCMD(spitter);

    this.timer = new Timer();
    timer.start();

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    indexerStop.schedule();
    llamaNeckStop.schedule();
    spitterForward.schedule();

    timer.reset();
  }

  @Override
  public void execute() {
    if (spitter.atSpeed()) {
      indexerForward.schedule();
      llamaNeckForward.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (llamaNeck.getUpperLimitSwitch() || llamaNeck.getLowerLimitSwitch()) {
      timer.reset();
    }
    return timer.hasElapsed(TIME_UNTIL_FINISHED);
  }
}
