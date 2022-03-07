package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Spitter2022;

import java.util.function.Supplier;

import org.apache.logging.log4j.Logger;

public class Spitter2022SetSpeedCMD extends CommandBase {
  private static final Logger LOGGER =
      RobotLogManager.getMainLogger(Spitter2022SetSpeedCMD.class.getName());
  private final Spitter2022 spitter;
  private final Supplier<Double> speed;

  public Spitter2022SetSpeedCMD(Spitter2022 spitter, Supplier<Double> speed) {
    this.spitter = spitter;
    this.speed = speed;

    addRequirements(spitter);
  }

  @Override
  public void initialize() {
    LOGGER.debug("Setting spitter forward");
  }

  @Override
  public void execute() {
    spitter.setVelocity(speed.get());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
