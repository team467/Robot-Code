package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class ReleaseCMD extends CommandBase {
  private final IntakeRelease intakerelease;
  private final Arm arm;
  private boolean needsDrop;
  private Timer timer = new Timer();

  public ReleaseCMD(IntakeRelease intakerelease, Arm arm) {
    this.intakerelease = intakerelease;
    this.arm = arm;

    addRequirements(intakerelease, arm);
  }

  @Override
  public void initialize() {
    needsDrop = true;
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if ((!arm.hasDropped())
        && intakerelease.haveCone()
        && intakerelease.wantsCone()
        && (arm.getExtention() >= 0.2 && arm.getRotation() >= 0.1)
        && !timer.hasElapsed(1.0)
        && needsDrop) {
      arm.drop();
      return;
    }
    needsDrop = false;
    intakerelease.release();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1.5);
  }
}
