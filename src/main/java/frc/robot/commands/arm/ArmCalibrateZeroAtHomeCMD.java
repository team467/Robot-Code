package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmCalibrateZeroAtHomeCMD extends Command {
  private final Arm arm;

  public ArmCalibrateZeroAtHomeCMD(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setCalibratedAssumeHomePosition();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
