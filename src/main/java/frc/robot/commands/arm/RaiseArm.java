package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class RaiseArm extends CommandBase {

  private final Arm arm;

  public RaiseArm(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.raise();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isFinished();
  }
}
