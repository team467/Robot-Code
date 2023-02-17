package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Led2023;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;

public class WantConeCMD extends CommandBase {
  private IntakeRelease intakerelease;
  private Led2023 ledStrip;

  public WantConeCMD(IntakeRelease intakerelease, Led2023 ledStrip) {
    this.intakerelease = intakerelease;
    this.ledStrip = ledStrip;
    addRequirements(ledStrip);
    addRequirements(intakerelease);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakerelease.setWants(Wants.CONE);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
