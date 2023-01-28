package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ExtendHigh extends CommandBase {
  private final Arm arm;

  public ExtendHigh(Arm arm) {
    this.arm = arm;

    addRequirements(this.arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // arm.extendAndRotate(0, 0);

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false; // TODO: arm.isStopped();
  }
}
