package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants.ArmPosition;

public class ArmPositionCMD extends CommandBase {
  private final Arm arm;
  private final ArmPosition armPosition;

  public ArmPositionCMD(Arm arm, ArmPosition armPosition) {
    this.arm = arm;
    this.armPosition = armPosition;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setTargetPositions(armPosition.extendSetpoint, armPosition.rotateSetpoint);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return arm.isFinished();
  }
}
