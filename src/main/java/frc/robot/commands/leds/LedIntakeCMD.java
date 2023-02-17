package frc.robot.commands.leds;

import frc.robot.subsystems.Led2023;
import frc.robot.subsystems.Led2023.COLORS_467;

public class LedIntakeCMD extends LedBaseCMD {
  public LedIntakeCMD(Led2023 ledStrip) {
    super(ledStrip);
  }

  @Override
  public void execute() {
    if (inputs.cubeLimitSwitch) {
      ledStrip.setColorMovingDown(COLORS_467.Blue.getColor(), COLORS_467.Black.getColor()); // Blue, black
    } else {
      ledStrip.setColorMovingDown(COLORS_467.Green.getColor(), COLORS_467.Black.getColor()); // Green, black
    }
  }
}
