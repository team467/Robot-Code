package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Climber2022;
import org.apache.logging.log4j.Logger;

public class Climber2022EnableCMD extends InstantCommand {
  private static final Logger LOGGER =
      RobotLogManager.getMainLogger(Climber2022EnableCMD.class.getName());
  private final Climber2022 climber;

  public Climber2022EnableCMD(Climber2022 climber) {
    this.climber = climber;

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.enable();
    LOGGER.debug("Climber enabled");
  }

  @Override
  public void end(boolean interrupted) {}
}
