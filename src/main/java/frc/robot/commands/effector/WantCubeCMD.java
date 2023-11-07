package frc.robot.commands.effector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.effector.Effector;
import frc.robot.subsystems.effector.Effector.Wants;

public class WantCubeCMD extends Command {
  private final Effector effector;

  public WantCubeCMD(Effector effector) {
    this.effector = effector;
    addRequirements(effector);
  }

  @Override
  public void execute() {
    effector.setWants(Wants.CUBE);
  }

  @Override
  public boolean isFinished() {
    return effector.wantsCube();
  }
}
