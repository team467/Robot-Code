package frc.robot.commands;

import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Trigger2022;

import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Trigger2022IdleCMD extends CommandBase {
  private final Trigger2022 trigger;

  private static final Logger LOGGER = RobotLogManager.getMainLogger(Trigger2022IdleCMD.class.getName());


  /**
   * Creates a new Trigger2022IdleCMD.
   *
   * @param trigger The subsystem used by this command.
   */
  public Trigger2022IdleCMD(Trigger2022 trigger) {
    this.trigger = trigger;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(trigger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LOGGER.info("Setting Trigger Slow Forward");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    trigger.idle();
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