package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ArmScoreCMD extends CommandBase {
  private final Arm arm;
  private double extendSetpoint;
  private double rotateSetpoint;

  public ArmScoreCMD(Arm arm, double setpoint, double rotateSetpoint) {
    this.arm = arm;
    this.extendSetpoint = setpoint;
    this.rotateSetpoint = rotateSetpoint;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.setTargetPositions(extendSetpoint, rotateSetpoint);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return arm.finished();
  }
}
