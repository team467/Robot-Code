package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Indexer2022;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Shooter2022;
import frc.robot.subsystems.Spitter2022;

import java.util.function.Supplier;

import org.apache.logging.log4j.Logger;

public class Shooter2022ShootSpeedCMD extends CommandBase {

  private static final Logger LOGGER =
      RobotLogManager.getMainLogger(Shooter2022ShootSpeedCMD.class.getName());

  private final double TIME_UNTIL_FINISHED = 0.5;

  private final LlamaNeck2022 llamaNeck;
  private final Indexer2022 indexer;
  private final Spitter2022 spitter;
  private final Supplier<Double> speed;

  private final Command llamaNeckStop;
  private final Command llamaNeckForward;

  private final Command indexerStop;
  private final Command indexerForward;

  private final Command spitterForward;

  private final Timer timer;

  public Shooter2022ShootSpeedCMD(
      Shooter2022 shooter, Supplier<Double> speed) {
    super();

    this.llamaNeck = shooter.llamaNeck2022;
    this.spitter = shooter.spitter2022;
    this.indexer = shooter.indexer2022;
    this.speed = speed;

    this.llamaNeckStop = new LlamaNeck2022StopCMD(llamaNeck);
    this.llamaNeckForward = new LlamaNeck2022ForwardCMD(llamaNeck);

    this.indexerStop = new Indexer2022StopCMD(indexer);
    this.indexerForward = new Indexer2022ForwardCMD(indexer);

    this.spitterForward = new Spitter2022SetSpeedCMD(spitter, speed);

    this.timer = new Timer();
    timer.start();

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    LOGGER.debug("Preparing to shoot, stop motors and start up spitter.");
    indexerStop.schedule();
    llamaNeckStop.schedule();
    spitterForward.schedule();

    timer.reset();
  }

  @Override
  public void execute() {
    if (spitter.isAtShootingSpeed()) {
      LOGGER.debug("Spitter is at shooting speed! Throwing balls into flywheel.");
      indexerForward.schedule();
      llamaNeckForward.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (llamaNeck.upperLimitSwitchIsPressed() || llamaNeck.lowerLimitSwitchIsPressed()) {
      // any switch still pressed, continue shooting.
      timer.reset();
    }
    // has TIME_UNTIL_FINISHED seconds occurred since no balls have touched any limit switch?
    return timer.hasElapsed(TIME_UNTIL_FINISHED);
  }
}
