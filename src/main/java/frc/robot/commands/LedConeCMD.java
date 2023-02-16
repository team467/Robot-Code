package frc.robot.commands;

import frc.robot.subsystems.Led2023;

public class LedConeCMD extends Led2023UpdateCMD {
  public LedConeCMD(Led2023 ledStrip) {
    super(ledStrip);
  }

  @Override
  void setColorPeriodic() {
    if (hasCone()) {
    setColorMovingUp(COLORS_467.Gold, COLORS_467.Black);
    } else {
      setColorMovingDown(COLORS_467.Gold, COLORS_467.Black);
    }
  }
}


