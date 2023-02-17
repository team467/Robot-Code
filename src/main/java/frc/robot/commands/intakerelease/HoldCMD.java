package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Led2023;
import frc.robot.subsystems.Led2023.COLORS_467;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class HoldCMD extends CommandBase {
  private IntakeRelease intakerelease;
  private Led2023 ledStrip;

  public HoldCMD(IntakeRelease intakerelease, Led2023 ledStrip) {
    this.intakerelease = intakerelease;

    addRequirements(intakerelease);
    this.ledStrip = ledStrip;
    addRequirements(ledStrip);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakerelease.hold();
    if (intakerelease.haveCube()) {
      ledStrip.setTop(COLORS_467.Blue);
      ledStrip.setBottom(COLORS_467.White);
    } else {
      ledStrip.setTop(COLORS_467.Green);
      ledStrip.setBottom(COLORS_467.White);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
