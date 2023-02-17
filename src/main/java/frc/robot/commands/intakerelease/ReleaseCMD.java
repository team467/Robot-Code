package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Led2023;
import frc.robot.subsystems.Led2023.COLORS_467;
import frc.robot.subsystems.intakerelease.IntakeRelease;

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
          COLORS_467.Blue.getColor(), COLORS_467.Black.getColor()); // Blue,black
    } else {
      ledStrip.setColorMovingUp(
          COLORS_467.Green.getColor(), COLORS_467.Black.getColor()); // Green, black
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
