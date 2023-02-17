package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class IntakeCMD extends CommandBase {
  private IntakeRelease intakerelease;

  public IntakeCMD(IntakeRelease intakerelease) {
    this.intakerelease = intakerelease;

    addRequirements(intakerelease);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakerelease.intake();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return intakerelease.haveCube() || intakerelease.haveCone();
  }
}
