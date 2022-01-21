package frc.robot.commands;

import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Intake2022;

import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Intake2022StopCMD extends CommandBase {
  private final Intake2022 intake;

  private static final Logger LOGGER = RobotLogManager.getMainLogger(Intake2022StopCMD.class.getName());

  /**
   * Creates a new Intake2022StopCMD.
   *
   * @param intake The subsystem used by this command.
   */
  public Intake2022StopCMD(Intake2022 intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LOGGER.info("Setting intake off");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      intake.intakeStop();
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