package frc.robot.commands.leds;

import frc.robot.subsystems.Led2023;
import frc.robot.subsystems.Led2023.COLORS_467;

public class LedWantsConeCMD extends LedBaseCMD {
  public LedWantsConeCMD(Led2023 ledStrip) {
    super(ledStrip);
  }

  @Override
  public void execute() {
    ledStrip.setBottom(COLORS_467.Green);
  }
}
