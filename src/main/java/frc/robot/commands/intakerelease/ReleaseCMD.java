package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.COLORS_467;

public class ReleaseCMD extends CommandBase {
  private IntakeRelease intakerelease;
  private Led2023 ledStrip;

  public ReleaseCMD(IntakeRelease intakerelease, Led2023 ledStrip) {
    this.intakerelease = intakerelease;

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
      ledStrip.setColorMovingUp(
          COLORS_467.Black.getColor(), COLORS_467.Blue.getColor()); // Blue,black
    } else if (intakerelease.haveCone()) {
      ledStrip.setColorMovingUp(
          COLORS_467.Black.getColor(), COLORS_467.Green.getColor()); // Green, black
    } else {
      ledStrip.setTop(COLORS_467.Gold);
      ledStrip.setTop(COLORS_467.Blue);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
