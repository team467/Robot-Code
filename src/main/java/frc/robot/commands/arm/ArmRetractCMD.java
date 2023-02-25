package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.led.Led2023;

public class ArmRetractCMD extends CommandBase {
  private Arm arm;
  private Led2023 ledStrip;

  public ArmRetractCMD(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setTargetPositionExtend(0.05);
  }

  @Override
  public void end(boolean interrupted) {
    arm.hold();
    ledStrip.defaultLights();
  }

  @Override
  public boolean isFinished() {
    return arm.isHolding();
  }
}
