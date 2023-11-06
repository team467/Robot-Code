package frc.robot.commands.effector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.effector.Effector;
import frc.robot.subsystems.effector.Effector.Wants;

public class WantConeCMD extends CommandBase {
  private final Effector effector;

  public WantConeCMD(Effector effector) {
    this.effector = effector;
    addRequirements(effector);
  }

  @Override
  public void execute() {
    effector.setWants(Wants.CONE);
  }

  @Override
  public boolean isFinished() {
    return effector.wantsCone();
  }
}
