package frc.robot.commands;

import frc.robot.subsystems.Led2023;

public class LedWantsCube extends Led2023UpdateCMD {
  /*/    public void initialize() {
  //        AddressableLED m_led = new AddressableLED(0);
      }
      public void execute() {

      }*/
  public LedWantsCube(Led2023 ledStrip) {
    super(ledStrip);
  }

  @Override
  void setColorPeriodic() {
    setTop(COLORS_467.Orange);
    setBottom(COLORS_467.Blue);
  }
}
