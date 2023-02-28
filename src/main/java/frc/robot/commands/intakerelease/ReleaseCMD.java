package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.COLORS_467;

public class ReleaseCMD extends CommandBase {
  private final IntakeRelease intakerelease;
  private final Led2023 ledStrip;

  public ReleaseCMD(IntakeRelease intakerelease, Led2023 ledStrip) {
    this.intakerelease = intakerelease;
    this.ledStrip = ledStrip;

    addRequirements(intakerelease, ledStrip);
  }

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
      ledStrip.setColorMovingDownTwoClr(COLORS_467.Gold.getColor(), COLORS_467.Purple.getColor());
    }
  }
}
