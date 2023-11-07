package frc.robot.commands.effector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.effector.Effector;

public class HoldCMD extends Command {
  private final Effector effector;
  private Timer timer = new Timer();

  public HoldCMD(Effector effector) {
    this.effector = effector;
    addRequirements(effector);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (effector.haveCube()
        && !effector.haveCone()
        && effector.wantsCube()
        && !timer.hasElapsed(3.0)) {
      effector.holdCube();
      if (effector.cubeLimitSwitch()) {
        timer.restart();
      }
    } else if (effector.haveCone() && effector.wantsCone() && !timer.hasElapsed(3.0)) {
      effector.holdCone();
      if (effector.coneLimitSwitch()) {
        timer.restart();
      }
    } else {
      effector.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  public boolean isFinished() {
    return false;
  }
}
