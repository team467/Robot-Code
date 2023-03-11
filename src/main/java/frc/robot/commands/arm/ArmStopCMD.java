package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.led.Led2023;

public class ArmStopCMD extends CommandBase {
  private final Arm arm;
  private final Led2023 ledStrip;

  public ArmStopCMD(Arm arm, Led2023 ledStrip) {
    this.arm = arm;
    this.ledStrip = ledStrip;

    addRequirements(arm, ledStrip);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.stop();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return arm.isStopped();
  }
}
