package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;

public class WantConeCMD extends CommandBase {
  private IntakeRelease intakerelease;

  public WantConeCMD(IntakeRelease intakerelease) {
    this.intakerelease = intakerelease;

    addRequirements(intakerelease);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakerelease.setWants(Wants.CONE);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
