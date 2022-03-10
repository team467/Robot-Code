package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Indexer2022;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Shooter2022;
import frc.robot.subsystems.Spitter2022;
import frc.robot.vision.HubTarget;
import org.apache.logging.log4j.Logger;

public class Shooter2022ShootTargetCMD extends CommandBase {

  private static final Logger LOGGER =
      RobotLogManager.getMainLogger(Shooter2022ShootTargetCMD.class.getName());

  private final double TIME_UNTIL_FINISHED = 1.5;
  private final double TIME_BETWEEN_BALLS = 0.3;

  private final LlamaNeck2022 llamaNeck;
  private final Indexer2022 indexer;
  private final Spitter2022 spitter;

  private final Command llamaNeckStop;
  private final Command llamaNeckForward;

  private final Command indexerStop;
  private final Command indexerForward;

  private final Command spitterTarget;
  private final Command spitterSpeed;

  private final Timer timer;
  private final Timer shotTimer;

  public Shooter2022ShootTargetCMD(
      Shooter2022 shooter,  double defaultDistance) {
    super();

    this.llamaNeck = shooter.llamaNeck2022;
    this.spitter = shooter.spitter2022;
    this.indexer = shooter.indexer2022;

    this.llamaNeckStop = new LlamaNeck2022StopCMD(llamaNeck);
    this.llamaNeckForward = new LlamaNeck2022ForwardCMD(llamaNeck);

    this.indexerStop = new Indexer2022StopCMD(indexer);
    this.indexerForward = new Indexer2022ForwardCMD(indexer);

    this.spitterTarget = new Spitter2022TargetCMD(spitter);
    this.spitterSpeed = new Spitter2022SetSpeedCMD(spitter, () -> Spitter2022.getFlywheelVelocity(defaultDistance));

    this.timer = new Timer();
    timer.start();

    this.shotTimer = new Timer();
    shotTimer.start();

    addRequirements(shooter);
  }

  public Shooter2022ShootTargetCMD(Shooter2022 shooter) {
    this(shooter, 0.6);
  }

  @Override
  public void initialize() {
    LOGGER.debug("Preparing to shoot, stop motors and start up spitter.");
    indexerStop.schedule();
    llamaNeckStop.schedule();
    spitterTarget.schedule();

    timer.reset();
  }

  @Override
  public void execute() {
    if (HubTarget.hasTarget()) {
      spitterTarget.schedule();
    } else {
      spitterSpeed.schedule();
    }

    if (spitter.isAtShootingSpeed()) {
      LOGGER.debug("Spitter is at shooting speed! Throwing balls into flywheel.");
      indexerForward.schedule();
    } else {
      indexerStop.schedule();
    }

    if (llamaNeck.upperLimitSwitchIsPressed()) {
      shotTimer.reset();
    } else {
      if (shotTimer.hasElapsed(TIME_BETWEEN_BALLS)) {
        llamaNeckForward.schedule();
      } else {
        llamaNeckStop.schedule();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (llamaNeck.upperLimitSwitchIsPressed() || llamaNeck.lowerLimitSwitchIsPressed() || !shotTimer.hasElapsed(TIME_BETWEEN_BALLS)) {
      // any switch still pressed, continue shooting.
      timer.reset();
    }
    // has TIME_UNTIL_FINISHED seconds occurred since no balls have touched any limit switch?
    return timer.hasElapsed(TIME_UNTIL_FINISHED);
  }
}