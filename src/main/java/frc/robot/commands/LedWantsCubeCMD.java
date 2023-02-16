package frc.robot.commands;

import frc.robot.subsystems.Led2023;

public class LedWantsCubeCMD extends Led2023UpdateCMD {
  /*/    public void initialize() {
  //        AddressableLED m_led = new AddressableLED(0);
      }
      public void execute() {

      }*/
  public LedWantsCubeCMD(Led2023 ledStrip) {
    super(ledStrip);
  }

  @Override
  void setColorPeriodic() {
    set(COLORS_467.Blue);
  }
}
