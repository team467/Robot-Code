package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.ColorScheme;

public class ArmCalibrateCMD extends CommandBase {
  private final Arm arm;
  private final Led2023 ledStrip;

  public ArmCalibrateCMD(Arm arm, Led2023 ledStrip) {
    this.arm = arm;
    this.ledStrip = ledStrip;

    addRequirements(arm, ledStrip);
  }

  @Override
  public void initialize() {
    arm.calibrate();
  }

  @Override
  public void execute() {
    ledStrip.setCmdColorScheme(ColorScheme.CALIBRATING);
  }

  @Override
  public void end(boolean interrupted) {
    if (arm.getIsCalibrated()) {
      ledStrip.setArmCalibrated();
    }
  }

  @Override
  public boolean isFinished() {
    return arm.isHolding();
  }
}
