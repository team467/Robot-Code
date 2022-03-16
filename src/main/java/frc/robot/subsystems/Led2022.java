package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.led.DoubleLEDStrip;
import frc.robot.led.LEDManager;

public class Led2022 extends SubsystemBase {
  private final DoubleLEDStrip ledStrip;

  public Led2022() {
    super();

    ledStrip = LEDManager.getInstance().createDoubleStrip(RobotConstants.get().led2022LedCount());

    for (int i = 0; i < ledStrip.getSize(); i++) {
      ledStrip.setRGB(i, 0, 0, 0);
    }
  }

  /**
   * Sets an LED to a specified color
   *
   * @param index the LED to set
   * @param color the color to set the LED to
   */
  public void setLED(int index, Color color) {
    ledStrip.setLED(index, color);
  }

  /**
   * Sets an LED to a specified 8 bit color
   *
   * @param index the LED to set
   * @param color the color to set the LED to
   */
  public void setLED(int index, Color8Bit color) {
    ledStrip.setLED(index, color);
  }

  /**
   * Sets an LED to a specified RGB color
   *
   * @param index the LED to set
   * @param r the red value to set
   * @param g the green value to set
   * @param b the blue value to set
   */
  public void setRGB(int index, int r, int g, int b) {
    ledStrip.setRGB(index, r, g, b);
  }

  /**
   * Sets an LED to a specified RGB color
   *
   * @param index the LED to set
   * @param h the h value to set
   * @param s the s value to set
   * @param v the v value to set
   */
  public void setHSV(int index, int h, int s, int v) {
    ledStrip.setHSV(index, h, s, v);
  }

  /**
   * Sets an LED to a specified RGB color
   *
   * @param index the LED to set
   * @param h the h value to set
   * @param s the s value to set
   * @param b the b value to set
   */
  public void setHSB(int index, float h, float s, float b) {
    ledStrip.setHSB(index, h, s, b);
  }

  /**
   * Sets an LED to a specified RGB color
   *
   * @param index the LED to set
   * @param h the h value to set
   * @param s the s value to set
   * @param b the b value to set
   */
  public void setHSB(int index, int h, int s, int b) {
    setHSB(index, h / 360f, s / 255f, b / 255f);
  }

  /** Updates the LED Strip */
  public void sendData() {
    ledStrip.update();
  }
}
