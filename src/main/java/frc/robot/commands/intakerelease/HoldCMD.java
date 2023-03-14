package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;

public class HoldCMD extends CommandBase {
  private final IntakeRelease intakerelease;

  public HoldCMD(IntakeRelease intakerelease) {
    this.intakerelease = intakerelease;
    addRequirements(intakerelease);
  }

  @Override
  public void execute() {
    if (intakerelease.haveCube()
        && !intakerelease.haveCone()
        && intakerelease.getWants() == Wants.CUBE) {
      intakerelease.holdCube();
    } else if (intakerelease.haveCone() && intakerelease.getWants() == Wants.CONE) {
      intakerelease.holdCone();
    } else {
      intakerelease.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  public boolean isFinished() {
    return intakerelease.wantsCone() || intakerelease.wantsCube();
  }
}
