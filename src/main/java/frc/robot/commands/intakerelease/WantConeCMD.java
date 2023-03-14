package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;

public class WantConeCMD extends CommandBase {
  private final IntakeRelease intakerelease;

  public WantConeCMD(IntakeRelease intakerelease) {
    this.intakerelease = intakerelease;
    addRequirements(intakerelease);
  }

  @Override
  public void execute() {
    intakerelease.setWants(Wants.CONE);
  }

  @Override
  public boolean isFinished() {
    return intakerelease.wantsCone();
  }
}
