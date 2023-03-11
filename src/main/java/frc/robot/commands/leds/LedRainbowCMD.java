package frc.robot.commands.leds;

import frc.robot.subsystems.led.Led2023;

public class LedRainbowCMD extends LedBaseCMD {

  public LedRainbowCMD(Led2023 ledStrip) {
    super(ledStrip);
  }

  @Override
  public void execute() {
    ledStrip.getColorScheme();
  }
}
