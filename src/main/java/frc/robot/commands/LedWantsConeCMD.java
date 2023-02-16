package frc.robot.commands;

import frc.robot.subsystems.Led2023;

public class LedWantsConeCMD extends Led2023UpdateCMD {
  /*/    public void initialize() {
  //        AddressableLED m_led = new AddressableLED(0);
      }
      public void execute() {

      }*/
  public LedWantsConeCMD(Led2023 ledStrip) {
    super(ledStrip);
  }

  @Override
  void setColorPeriodic() {
    setTop(COLORS_467.Gold);
    setBottom(COLORS_467.Green);
  }
}
