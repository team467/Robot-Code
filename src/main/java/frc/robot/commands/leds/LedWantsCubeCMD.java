package frc.robot.commands.leds;

import frc.robot.subsystems.Led2023;
import frc.robot.subsystems.Led2023.COLORS_467;

public class LedWantsCubeCMD extends LedBaseCMD {
  public boolean wantCube;

  public LedWantsCubeCMD(Led2023 ledStrip) {
    super(ledStrip);
  }

  @Override
  public void execute() {
    ledStrip.set(COLORS_467.Blue);
  }
}
