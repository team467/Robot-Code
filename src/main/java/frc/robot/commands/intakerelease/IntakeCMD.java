package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Led2023;
import frc.robot.subsystems.Led2023.COLORS_467;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;

public class IntakeCMD extends CommandBase {
  private IntakeRelease intakerelease;
  private Led2023 ledStrip;
  private Wants wants;

  public IntakeCMD(IntakeRelease intakerelease, Led2023 ledStrip) {
    this.intakerelease = intakerelease;
    this.ledStrip = ledStrip;
    addRequirements(ledStrip);

    addRequirements(intakerelease);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakerelease.intake();
    if (Wants.CUBE == wants) {
      ledStrip.setColorMovingDown(
          COLORS_467.Blue.getColor(), COLORS_467.Black.getColor()); // Blue, black
    } else {
      ledStrip.setColorMovingDown(
          COLORS_467.Green.getColor(), COLORS_467.Black.getColor()); // Green, black
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return intakerelease.haveCube() || intakerelease.haveCone();
  }
}
