package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.ColorScheme;

public class WantCubeCMD extends CommandBase {
  private final IntakeRelease intakerelease;
  private final Led2023 ledStrip;

  public WantCubeCMD(IntakeRelease intakerelease, Led2023 ledStrip) {
    this.intakerelease = intakerelease;
    this.ledStrip = ledStrip;
    addRequirements(intakerelease);
  }

  @Override
  public void initialize() {
    intakerelease.setWants(Wants.CUBE);
    ledStrip.setCmdColorScheme(ColorScheme.WANT_CUBE);
  }

  @Override
  public boolean isFinished() {
    return intakerelease.wantsCube();
  }
}
