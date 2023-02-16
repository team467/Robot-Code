package frc.robot.commands;

import frc.robot.subsystems.Led2023;

public class LedWantsConeCMD extends Led2023UpdateCMD {
  public LedWantsConeCMD(Led2023 ledStrip) {
    super(ledStrip);
  }

  @Override
  void setColorPeriodic() {
    setBottom(COLORS_467.Green);
  }
}
