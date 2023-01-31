package frc.robot.commands;

import frc.robot.subsystems.Led2023;

public class LedBlueGold extends Led2023UpdateCMD {

  public LedBlueGold(Led2023 ledStrip) {
    super(ledStrip);
  }

  @Override
  void setColorPeriodic() {
    setTop(COLORS_467.Gold);
    setBottom(COLORS_467.Blue);
  }
}
