package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class ArmSetPosition extends CommandBase {
  private Arm arm;
  private IntakeRelease intakeRelease;
  private double rotatePosition;
  private double extentionPosition;

  public ArmSetPosition(
      Arm arm, IntakeRelease intakeRelease, double rotatePosition, double extentionPosition) {
    this.arm = arm;
    this.intakeRelease = intakeRelease;
    this.rotatePosition = rotatePosition;
    this.extentionPosition = extentionPosition;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.setTargetPositions(extentionPosition, rotatePosition);
  }
}
