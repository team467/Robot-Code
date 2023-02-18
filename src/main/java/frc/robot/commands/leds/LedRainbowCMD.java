package frc.robot.commands.leds;

import frc.robot.subsystems.Led2023;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class LedRainbowCMD extends LedBaseCMD {
  private IntakeRelease intakerelease;

  public LedRainbowCMD(Led2023 ledStrip, IntakeRelease intakerelease) {
    super(ledStrip);
    this.intakerelease = intakerelease;
    addRequirements(intakerelease);
  }

  @Override
  public void execute() {
    ledStrip.defaultLights();
  }
}
