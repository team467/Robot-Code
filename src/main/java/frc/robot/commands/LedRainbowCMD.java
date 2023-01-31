package frc.robot.commands;

import frc.robot.subsystems.Led2023;

public class LedRainbowCMD extends Led2023UpdateCMD {

  public LedRainbowCMD(Led2023 ledStrip) {
    super(ledStrip);
  }

  @Override
  void setColorPeriodic() {
    setRainbowMovingDown();
  }
}
