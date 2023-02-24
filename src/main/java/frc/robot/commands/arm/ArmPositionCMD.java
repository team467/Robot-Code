package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants.ArmPosition;

public class ArmPositionCMD extends CommandBase {
  private final Arm arm;
  private final Supplier<ArmPosition> armPositionSupplier;

  public ArmPositionCMD(Arm arm, ArmPosition armPosition) {
    this.arm = arm;
    this.armPositionSupplier = ()->armPosition;
    addRequirements(arm);
  }

  public ArmPositionCMD(Arm arm, Supplier<ArmPosition> armPosition) {
    this.arm = arm;
    this.armPositionSupplier = armPosition;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    ArmPosition armPosition = armPositionSupplier.get();
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
