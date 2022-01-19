package frc.robot.commands;

import frc.robot.subsystems.Intake2022;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Intake2022InCMD extends CommandBase {
  private final Intake2022 intake;

  /**
   * Creates a new Intake2022InCMD.
   *
   * @param intake The subsystem used by this command.
   */
  public Intake2022InCMD(Intake2022 intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      intake.intakeIn();
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