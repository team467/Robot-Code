package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;

public class WantCubeCMD extends CommandBase {
  private final IntakeRelease intakerelease;

  public WantCubeCMD(IntakeRelease intakerelease) {
    this.intakerelease = intakerelease;
    addRequirements(intakerelease);
  }

  @Override
  public void execute() {
    intakerelease.setWants(Wants.CUBE);
  }

  @Override
  public boolean isFinished() {
    return intakerelease.wantsCube();
  }
}
