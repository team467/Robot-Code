package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Indexer2022;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Shooter2022;
import frc.robot.subsystems.Spitter2022;

import java.util.function.Supplier;

import org.apache.logging.log4j.Logger;

public class Shooter2022IdleSpinupCMD extends CommandBase {

  private static final Logger LOGGER =
      RobotLogManager.getMainLogger(Shooter2022IdleSpinupCMD.class.getName());

  private final LlamaNeck2022 llamaNeck;
  private final Indexer2022 indexer;
  private final Spitter2022 spitter;
  private final Supplier<Double> speed;

  private final Command llamaNeckStop;
  private final Command llamaNeckIdle;

  private final Command indexerStop;
  private final Command indexerIdle;

  private final Command spitterStop;
  private final Command spitterSpeed;

  public Shooter2022IdleSpinupCMD(Shooter2022 shooter, Supplier<Double> speed) {
    super();

    this.llamaNeck = shooter.llamaNeck2022;
    this.spitter = shooter.spitter2022;
    this.indexer = shooter.indexer2022;
    this.speed = speed;

    this.llamaNeckStop = new LlamaNeck2022StopCMD(llamaNeck);
    this.llamaNeckIdle = new LlamaNeck2022IdleCMD(llamaNeck);

    this.indexerStop = new Indexer2022StopCMD(indexer);
    this.indexerIdle = new Indexer2022IdleCMD(indexer);

    this.spitterStop = new Spitter2022StopCMD(spitter);
    this.spitterSpeed = new Spitter2022SetSpeedCMD(spitter, speed);

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    LOGGER.debug("Idling system...");
    indexerIdle.schedule();
    llamaNeckIdle.schedule();
    spitterStop.schedule();
  }

  @Override
  public void execute() {
    if (llamaNeck.upperLimitSwitchIsPressed()) {
      LOGGER.debug("Upper limit switch was activated. Stop indexer.");
      indexerStop.schedule();
      spitterSpeed.schedule();

      if (llamaNeck.lowerLimitSwitchIsPressed()) {
        LOGGER.debug("Lower limit switch was activated. Stop llama neck.");
        llamaNeckStop.schedule();
      }
    } else {
      indexerIdle.schedule();
      llamaNeckIdle.schedule();
      spitterStop.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
