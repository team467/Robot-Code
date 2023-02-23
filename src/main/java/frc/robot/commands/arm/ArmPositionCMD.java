package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ArmPositionCMD extends CommandBase {
  private final Arm arm;
  private double extendSetpoint;
  private double rotateSetpoint;

  public ArmPositionCMD(Arm arm, double extendSetpoint, double rotateSetpoint) {
    this.arm = arm;
    this.extendSetpoint = extendSetpoint;
    this.rotateSetpoint = rotateSetpoint;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setTargetPositions(extendSetpoint, rotateSetpoint);
  }


  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return arm.isFinished();
  }
}
