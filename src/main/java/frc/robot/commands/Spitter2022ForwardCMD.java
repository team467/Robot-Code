package frc.robot.commands;

import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Spitter2022;

import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Spitter2022ForwardCMD extends CommandBase {
  private final Spitter2022 spitter;

  private static final Logger LOGGER = RobotLogManager.getMainLogger(Spitter2022ForwardCMD.class.getName());


  /**
   * Creates a new Spitter2022ForwardCMD.
   *
   * @param spitter The subsystem used by this command.
   */
  public Spitter2022ForwardCMD(Spitter2022 spitter) {
    this.spitter = spitter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spitter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LOGGER.info("Setting spitter forward");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spitter.forward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}