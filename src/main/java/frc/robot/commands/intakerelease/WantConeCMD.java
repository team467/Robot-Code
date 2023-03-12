package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.ColorScheme;

public class WantConeCMD extends CommandBase {
  private final IntakeRelease intakerelease;
  private final Led2023 ledStrip;

  public WantConeCMD(IntakeRelease intakerelease, Led2023 ledStrip) {
    this.intakerelease = intakerelease;
    this.ledStrip = ledStrip;
    addRequirements(intakerelease);
  }

  @Override
  public void initialize() {
    ledStrip.setCmdColorScheme(ColorScheme.WANT_CONE);
  }

  @Override
  public void execute() {
    intakerelease.setWants(Wants.CONE);
  }

  @Override
  public boolean isFinished() {
    return intakerelease.wantsCone();
  }
}
