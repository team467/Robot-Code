package frc.robot.commands;

import frc.robot.subsystems.Led2023;

public class LedWantsCone extends Led2023UpdateCMD {
  /*/    public void initialize() {
  //        AddressableLED m_led = new AddressableLED(0);
      }
      public void execute() {

      }*/
  public LedWantsCone(Led2023 ledStrip) {
    super(ledStrip);
  }

  @Override
  void setColorPeriodic() {
    setTop(COLORS_467.Green);
    setBottom(COLORS_467.Green);
  }
}
