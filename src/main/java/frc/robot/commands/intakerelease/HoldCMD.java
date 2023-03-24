package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class HoldCMD extends CommandBase {
  private final IntakeRelease intakerelease;
  private Timer timer = new Timer();

  public HoldCMD(IntakeRelease intakerelease) {
    this.intakerelease = intakerelease;
    addRequirements(intakerelease);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (intakerelease.haveCube()
        && !intakerelease.haveCone()
        && intakerelease.wantsCube()
        && !timer.hasElapsed(3.0)) {
      intakerelease.holdCube();
      if (intakerelease.cubeLimitSwitch()) {
        timer.restart();
      }
    } else if (intakerelease.haveCone() && intakerelease.wantsCone() && !timer.hasElapsed(3.0)) {
      intakerelease.holdCone();
      if (intakerelease.coneLimitSwitch()) {
        timer.restart();
      }
    } else {
      intakerelease.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  public boolean isFinished() {
    return false;
  }
}
