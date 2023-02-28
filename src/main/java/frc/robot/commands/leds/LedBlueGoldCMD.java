package frc.robot.commands.leds;

import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.COLORS_467;

public class LedBlueGoldCMD extends LedBaseCMD {

  public LedBlueGoldCMD(Led2023 ledStrip) {
    super(ledStrip);
  }

  @Override
  public void execute() {
    ledStrip.setTop(COLORS_467.Gold);
    ledStrip.setBottom(COLORS_467.Blue);
  }
}
