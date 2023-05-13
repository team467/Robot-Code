package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;

public class IntakeCMD extends CommandBase {
  private final IntakeRelease intakerelease;

  public IntakeCMD(IntakeRelease intakerelease) {
    this.intakerelease = intakerelease;

    addRequirements(intakerelease);
  }

  @Override
  public void initialize() {
    intakerelease.resetHas();
  }

  @Override
  public void execute() {
    intakerelease.intake();
  }

  @Override
  public void end(boolean interrupted) {
    if (intakerelease.getWants() == Wants.CONE) {
      intakerelease.holdCone();
    } else {
      intakerelease.holdCube();
    }
  }

  @Override
  public boolean isFinished() {
    return (intakerelease.getWants() == Wants.CUBE && intakerelease.haveCube())
        || intakerelease.haveCone();
  }
}
