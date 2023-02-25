package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.COLORS_467;

public class IntakeCMD extends CommandBase {
  private final IntakeRelease intakerelease;
  private final Led2023 ledStrip;

  public IntakeCMD(IntakeRelease intakerelease, Led2023 ledStrip) {
    this.intakerelease = intakerelease;
    this.ledStrip = ledStrip;

    addRequirements(intakerelease, ledStrip);
  }

  @Override
  public void execute() {
    intakerelease.intake();
    if (intakerelease.getWants() == Wants.CUBE) {
      ledStrip.setColorMovingUp(
          COLORS_467.White.getColor(), COLORS_467.Purple.getColor()); // Purple, black
    } else if (intakerelease.getWants() == Wants.CONE) {
      ledStrip.setColorMovingUp(
          COLORS_467.White.getColor(), COLORS_467.Yellow.getColor()); // Gold, black
    } else {
      ledStrip.setColorMovingUpTwoClr(COLORS_467.Purple.getColor(), COLORS_467.Yellow.getColor());
    }
  }

  @Override
  public void end(boolean interrupted) {
    ledStrip.defaultLights();
  }

  @Override
  public boolean isFinished() {
    return (intakerelease.getWants() == Wants.CUBE && intakerelease.haveCube())
        || intakerelease.haveCone();
  }
}
