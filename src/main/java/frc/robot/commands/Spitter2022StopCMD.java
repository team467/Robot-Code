package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Spitter2022;
import org.apache.logging.log4j.Logger;

public class Spitter2022StopCMD extends CommandBase {
  private static final Logger LOGGER =
      RobotLogManager.getMainLogger(Spitter2022StopCMD.class.getName());
  private final Spitter2022 spitter;

  public Spitter2022StopCMD(Spitter2022 spitter) {
    this.spitter = spitter;
    addRequirements(spitter);
  }

  @Override
  public void initialize() {
    LOGGER.debug("Setting spitter stopped");
  }

  @Override
  public void execute() {
    spitter.stop();
  }

  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
