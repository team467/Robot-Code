package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.COLORS_467;

public class ReleaseCMD extends CommandBase {
  private IntakeRelease intakerelease;
  private Led2023 ledStrip;
  private HoldCMD holdCMD;

  public ReleaseCMD(IntakeRelease intakerelease, Led2023 ledStrip, HoldCMD holdCMD) {
    this.intakerelease = intakerelease;
    this.holdCMD = holdCMD;
    addRequirements(intakerelease);
    this.ledStrip = ledStrip;
    addRequirements(ledStrip);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakerelease.release();
    if (intakerelease.haveCube()) {
      ledStrip.setColorMovingDown(
          COLORS_467.Black.getColor(), COLORS_467.Purple.getColor()); // Purple,black
    } else if (intakerelease.haveCone()) {
      ledStrip.setColorMovingDown(
          COLORS_467.Black.getColor(), COLORS_467.Gold.getColor()); // Gold, black
    } else {
      ledStrip.setBottom(COLORS_467.Purple);
      ledStrip.setTop(COLORS_467.Gold);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
