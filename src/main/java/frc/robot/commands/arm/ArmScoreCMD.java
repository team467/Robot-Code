package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ArmScoreCMD extends CommandBase {
  private final Arm arm;
  private double setpoint;

  public ArmScoreCMD(Arm arm, double setpoint) {
    this.arm = arm;
    this.setpoint = setpoint;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.setTargetPositions(setpoint, 0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return arm.finished();
  }
}
