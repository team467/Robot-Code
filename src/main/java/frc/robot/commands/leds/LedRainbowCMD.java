package frc.robot.commands.leds;

import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;

public class LedRainbowCMD extends LedBaseCMD {
  private IntakeRelease intakerelease;
  private Led2023 ledStrip;

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
