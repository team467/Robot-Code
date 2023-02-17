package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class ReleaseCMD extends CommandBase {
  private IntakeRelease intakerelease;

  public ReleaseCMD(IntakeRelease intakerelease) {
    this.intakerelease = intakerelease;

    addRequirements(intakerelease);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakerelease.release();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
