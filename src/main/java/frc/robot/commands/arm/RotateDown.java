package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class RotateDown extends CommandBase {
  private final Arm arm;

  public RotateDown(Arm arm) {
    this.arm = arm;

    addRequirements(this.arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // arm.rotateTargetDegrees;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return arm.isStopped();
  }
}
