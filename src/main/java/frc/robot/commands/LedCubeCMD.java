package frc.robot.commands;
  
import frc.robot.subsystems.Led2023;

public class LedCubeCMD extends Led2023UpdateCMD {
  public LedCubeCMD(Led2023 ledStrip) {
    super(ledStrip);
  }

  @Override
  void setColorPeriodic() {
    if (hasCube()) {
    setColorMovingUp(COLORS_467.Blue, COLORS_467.Black);
    } else {
      setColorMovingDown(COLORS_467.Blue, COLORS_467.Black);
    }
  }
}

