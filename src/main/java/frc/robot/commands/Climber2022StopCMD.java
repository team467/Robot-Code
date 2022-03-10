package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Climber2022;
import org.apache.logging.log4j.Logger;

public class Climber2022StopCMD extends CommandBase {
  private static final Logger LOGGER =
      RobotLogManager.getMainLogger(Climber2022StopCMD.class.getName());
  private final Climber2022 climber;

  public Climber2022StopCMD(Climber2022 climber) {
    this.climber = climber;

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    LOGGER.debug("Stopping climber");
  }

  @Override
  public void execute() {
    climber.stop();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
