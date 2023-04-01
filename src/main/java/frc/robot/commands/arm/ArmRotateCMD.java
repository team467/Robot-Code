package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ArmRotateCMD extends CommandBase {
  private Arm arm;
  private double rotatePosition;

  public ArmRotateCMD(
      Arm arm, double rotatePosition) {
    this.arm = arm;
    this.rotatePosition = rotatePosition;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.setTargetPositionRotate(rotatePosition);
  }
}
