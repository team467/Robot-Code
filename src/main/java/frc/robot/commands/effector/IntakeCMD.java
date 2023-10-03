package frc.robot.commands.effector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.effector.Effector;
import frc.robot.subsystems.effector.Effector.Wants;

public class IntakeCMD extends CommandBase {
  private final Effector effector;

  public IntakeCMD(Effector effector) {
    this.effector = effector;

    addRequirements(effector);
  }

  @Override
  public void initialize() {
    effector.resetHas();
  }

  @Override
  public void execute() {
    effector.intake();
  }

  @Override
  public void end(boolean interrupted) {
    if (effector.getWants() == Wants.CONE) {
      effector.holdCone();
    } else {
      effector.holdCube();
    }
  }

  @Override
  public boolean isFinished() {
    return (effector.getWants() == Wants.CUBE && effector.haveCube())
        || effector.haveCone();
  }
}
