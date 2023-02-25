package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;

public class ArmManualRetractCMD extends CommandBase {
  private Arm arm;
  private Led2023 ledStrip;
  private IntakeRelease intakerelease;

  public ArmManualRetractCMD(Arm arm, IntakeRelease intakerelease, Led2023 ledStrip) {
    this.arm = arm;
    this.ledStrip = ledStrip;
    this.intakerelease = intakerelease;
    addRequirements(arm, intakerelease, ledStrip);
  }

  @Override
  public void initialize() {
    arm.manualExtend(-1);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    arm.hold();
  }
}
