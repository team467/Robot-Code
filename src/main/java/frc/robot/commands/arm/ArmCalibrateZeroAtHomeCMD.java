package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.led.Led2023;

public class ArmCalibrateZeroAtHomeCMD extends CommandBase {
  private final Arm arm;
  private final Led2023 ledStrip;

  public ArmCalibrateZeroAtHomeCMD(Arm arm, Led2023 ledStrip) {
    this.arm = arm;
    this.ledStrip = ledStrip;

    addRequirements(arm, ledStrip);
  }

  @Override
  public void initialize() {
    if (!arm.isCalibrated()) {
      arm.setCalibratedAssumeHomePosition();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
