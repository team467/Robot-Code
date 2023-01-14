package frc.lib.input;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

// TODO: add javadoc comments (very long)

/** A factory to generate commands to run on the custom controller. */
public class ControllerCommandFactory {

  private ControllerCommandFactory() {}

  /**
   * Set the color of a single LED.
   *
   * @param index the LED to set the speed of
   * @param r the red value
   * @param g the green value
   * @param b the blue value
   * @return a command to set the color
   */
  public static byte[] setLEDRGB(int index, int r, int g, int b) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Single,
      (byte) index,
      ControllerDataProtocol.LightingValue.LedBaseColor,
      (byte) r,
      (byte) g,
      (byte) b
    };
  }

  /**
   * Set the color of a single LED.
   *
   * @param index the LED to set the speed of
   * @param color the color to set the LEDs
   * @return a command to set the color
   */
  public static byte[] setLEDBaseColor(int index, Color8Bit color) {
    return setLEDRGB(index, color.red, color.green, color.blue);
  }

  /**
   * Set the color of a single LED.
   *
   * @param index the LED to set the speed of
   * @param color the color to set the LEDs
   * @return a command to set the color
   */
  public static byte[] setLEDBaseColor(int index, Color color) {
    return setLEDBaseColor(index, new Color8Bit(color));
  }

  /**
   * Set the color of a single LED.
   *
   * @param index the LED to set the speed of
   * @param h the hue value
   * @param s the saturation value
   * @param b the brightness value
   * @return a command to set the color
   */
  public static byte[] setLEDHSB(int index, float h, float s, float b) {
    java.awt.Color outColor = java.awt.Color.getHSBColor(h, s, b);
    return setLEDRGB(index, outColor.getRed(), outColor.getGreen(), outColor.getBlue());
  }

  /**
   * Set the color of a single LED.
   *
   * @param index the LED to set the speed of
   * @param h the hue value
   * @param s the saturation value
   * @param b the brightness value
   * @return a command to set the color
   */
  public static byte[] setLEDHSB(int index, int h, int s, int b) {
    return setLEDHSB(index, h / 360f, s / 255f, b / 255f);
  }

  /**
   * Set the color of a LED range.
   *
   * @param startLED the first LED to modify
   * @param endLED the last LED to modify
   * @param r the red value
   * @param g the green value
   * @param b the blue value
   * @return a command to set the color
   */
  public static byte[] setLEDRangeRGB(int startLED, int endLED, int r, int g, int b) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Multiple,
      (byte) startLED,
      (byte) endLED,
      ControllerDataProtocol.LightingValue.LedBaseColor,
      (byte) r,
      (byte) g,
      (byte) b
    };
  }

  /**
   * Set the color of a LED range.
   *
   * @param startLED the first LED to modify
   * @param endLED the last LED to modify
   * @param color the color to set the LEDs
   * @return a command to set the color
   */
  public static byte[] setLEDRangeBaseColor(int startLED, int endLED, Color8Bit color) {
    return setLEDRangeRGB(startLED, endLED, color.red, color.green, color.blue);
  }

  /**
   * Set the color of a LED range.
   *
   * @param startLED the first LED to modify
   * @param endLED the last LED to modify
   * @param color the color to set the LEDs
   * @return a command to set the color
   */
  public static byte[] setLEDRangeBaseColor(int startLED, int endLED, Color color) {
    return setLEDRangeBaseColor(startLED, endLED, new Color8Bit(color));
  }

  /**
   * Set the color of a LED range.
   *
   * @param startLED the first LED to modify
   * @param endLED the last LED to modify
   * @param h the hue value
   * @param s the saturation value
   * @param b the brightness value
   * @return a command to set the color
   */
  public static byte[] setLEDRangeHSB(int startLED, int endLED, float h, float s, float b) {
    java.awt.Color outColor = java.awt.Color.getHSBColor(h, s, b);
    return setLEDRangeRGB(
        startLED, endLED, outColor.getRed(), outColor.getGreen(), outColor.getBlue());
  }

  /**
   * Set the color of a LED range.
   *
   * @param startLED the first LED to modify
   * @param endLED the last LED to modify
   * @param h the hue value
   * @param s the saturation value
   * @param b the brightness value
   * @return a command to set the color
   */
  public static byte[] setLEDRangeHSB(int startLED, int endLED, int h, int s, int b) {
    return setLEDRangeHSB(startLED, endLED, h / 360f, s / 255f, b / 255f);
  }

  /**
   * Set the color of a LED section.
   *
   * @param section the section to modify
   * @param r the red value
   * @param g the green value
   * @param b the blue value
   * @return a command to set the color
   */
  public static byte[] setLEDSectionRGB(int section, int r, int g, int b) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Section,
      (byte) section,
      ControllerDataProtocol.LightingValue.LedBaseColor,
      (byte) r,
      (byte) g,
      (byte) b
    };
  }

  /**
   * Set the color of a LED section.
   *
   * @param section the section to modify
   * @param color the color to set the LEDs
   * @return a command to set the color
   */
  public static byte[] setLEDSectionBaseColor(int section, Color8Bit color) {
    return setLEDSectionRGB(section, color.red, color.green, color.blue);
  }

  /**
   * Set the color of a LED section.
   *
   * @param section the section to modify
   * @param color the color to set the LEDs
   * @return a command to set the color
   */
  public static byte[] setLEDSectionBaseColor(int section, Color color) {
    return setLEDSectionBaseColor(section, new Color8Bit(color));
  }

  /**
   * Set the color of a LED range.
   *
   * @param section the section to modify
   * @param h the hue value
   * @param s the saturation value
   * @param b the brightness value
   * @return a command to set the color
   */
  public static byte[] setLEDSectionHSB(int section, float h, float s, float b) {
    java.awt.Color outColor = java.awt.Color.getHSBColor(h, s, b);
    return setLEDSectionRGB(section, outColor.getRed(), outColor.getGreen(), outColor.getBlue());
  }

  /**
   * Set the color of a LED range.
   *
   * @param section the section to modify
   * @param h the hue value
   * @param s the saturation value
   * @param b the brightness value
   * @return a command to set the color
   */
  public static byte[] setLEDSectionHSB(int section, int h, int s, int b) {
    return setLEDSectionHSB(section, h / 360f, s / 255f, b / 255f);
  }

  /**
   * Set the color of all LEDs.
   *
   * @param r the red value
   * @param g the green value
   * @param b the blue value
   * @return a command to set the color
   */
  public static byte[] setAllLEDRGB(int r, int g, int b) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.All,
      ControllerDataProtocol.LightingValue.LedBaseColor,
      (byte) r,
      (byte) g,
      (byte) b
    };
  }

  /**
   * Set the color of all LEDs.
   *
   * @param color the color to set the LEDs
   * @return a command to set the color
   */
  public static byte[] setAllLEDBaseColor(Color8Bit color) {
    return setAllLEDRGB(color.red, color.green, color.blue);
  }

  /**
   * Set the color of all LEDs.
   *
   * @param color the color to set the LEDs
   * @return a command to set the color
   */
  public static byte[] setAllLEDBaseColor(Color color) {
    return setAllLEDBaseColor(new Color8Bit(color));
  }

  /**
   * Set the color of all LEDs.
   *
   * @param h the hue value
   * @param s the saturation value
   * @param b the brightness value
   * @return a command to set the color
   */
  public static byte[] setAllLEDHSB(float h, float s, float b) {
    java.awt.Color outColor = java.awt.Color.getHSBColor(h, s, b);
    return setAllLEDRGB(outColor.getRed(), outColor.getGreen(), outColor.getBlue());
  }

  /**
   * Set the color of all LEDs.
   *
   * @param h the hue value
   * @param s the saturation value
   * @param b the brightness value
   * @return a command to set the color
   */
  public static byte[] setAllLEDHSB(int h, int s, int b) {
    return setAllLEDHSB(h / 360f, s / 255f, b / 255f);
  }

  public static byte[] setLEDEffect(int index, int effect) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Single,
      (byte) index,
      ControllerDataProtocol.LightingValue.LedEffect,
      (byte) effect
    };
  }

  public static byte[] setLEDRangeEffect(int startLED, int endLED, int effect) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Multiple,
      (byte) startLED,
      (byte) endLED,
      ControllerDataProtocol.LightingValue.LedEffect,
      (byte) effect
    };
  }

  public static byte[] setLEDSectionEffect(int section, int effect) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Section,
      (byte) section,
      ControllerDataProtocol.LightingValue.LedEffect,
      (byte) effect
    };
  }

  public static byte[] setAllLEDEffect(int effect) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.All,
      ControllerDataProtocol.LightingValue.LedEffect,
      (byte) effect
    };
  }

  public static byte[] setLEDRangeEffectSpaced(int startLED, int endLED, int effect) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Multiple,
      (byte) startLED,
      (byte) endLED,
      ControllerDataProtocol.LightingValue.LedEffectSpaced,
      (byte) effect
    };
  }

  public static byte[] setLEDSectionEffectSpaced(int section, int effect) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Section,
      (byte) section,
      ControllerDataProtocol.LightingValue.LedEffectSpaced,
      (byte) effect
    };
  }

  public static byte[] setAllLEDEffectSpaced(int effect) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.All,
      ControllerDataProtocol.LightingValue.LedEffectSpaced,
      (byte) effect
    };
  }

  /**
   * Set the offset of a single LED.
   *
   * @param index the LED to set the speed of
   * @param offset the offset to set
   * @return a command to set the speed
   */
  public static byte[] setLEDOffset(int index, int offset) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Single,
      (byte) index,
      ControllerDataProtocol.LightingValue.LedOffset,
      (byte) offset
    };
  }

  public static byte[] setLEDRangeOffset(int startLED, int endLED, int offset) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Multiple,
      (byte) startLED,
      (byte) endLED,
      ControllerDataProtocol.LightingValue.LedOffset,
      (byte) offset
    };
  }

  public static byte[] setLEDSectionOffset(int section, int offset) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Section,
      (byte) section,
      ControllerDataProtocol.LightingValue.LedOffset,
      (byte) offset
    };
  }

  public static byte[] setAllLEDOffset(int offset) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.All,
      ControllerDataProtocol.LightingValue.LedOffset,
      (byte) offset
    };
  }

  /**
   * Set the speed of a single LED.
   *
   * @param index the LED to set the speed of
   * @param speed the speed out of 255
   * @return a command to set the speed
   */
  public static byte[] setLEDSpeed(int index, int speed) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Single,
      (byte) index,
      ControllerDataProtocol.LightingValue.LedSpeed,
      (byte) speed
    };
  }

  /**
   * Set the brightness of the LEDs from startLED to endLED.
   *
   * @param startLED the first LED to set the speed of
   * @param endLED the last LED to set the speed of
   * @param speed the speed out of 255
   * @return a command to set the speed
   */
  public static byte[] setLEDRangeSpeed(int startLED, int endLED, int speed) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Multiple,
      (byte) startLED,
      (byte) endLED,
      ControllerDataProtocol.LightingValue.LedSpeed,
      (byte) speed
    };
  }

  /**
   * Set the speed of the LEDs in a section.
   *
   * @param section the section to modify
   * @param speed the speed out of 255
   * @return a command to set the speed
   */
  public static byte[] setLEDSectionSpeed(int section, int speed) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Section,
      (byte) section,
      ControllerDataProtocol.LightingValue.LedSpeed,
      (byte) speed
    };
  }

  /**
   * Set the speed of every LED.
   *
   * @param speed the speed out of 255
   * @return a command to set the speed
   */
  public static byte[] setAllLEDSpeed(int speed) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.All,
      ControllerDataProtocol.LightingValue.LedSpeed,
      (byte) speed
    };
  }

  /**
   * Set the brightness of a single LED.
   *
   * @param index the LED to set the brightness of
   * @param brightness the brightness out of 255
   * @return a command to set the brightness
   */
  public static byte[] setLEDBrightness(int index, int brightness) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Single,
      (byte) index,
      ControllerDataProtocol.LightingValue.LedBrightness,
      (byte) brightness
    };
  }

  /**
   * Set the brightness of the LEDs from startLED to endLED.
   *
   * @param startLED the first LED to set the brightness of
   * @param endLED the last LED to set the brightness of
   * @param brightness the brightness out of 255
   * @return a command to set the brightness
   */
  public static byte[] setLEDRangeBrightness(int startLED, int endLED, int brightness) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Multiple,
      (byte) startLED,
      (byte) endLED,
      ControllerDataProtocol.LightingValue.LedBrightness,
      (byte) brightness
    };
  }

  /**
   * Set the brightness of the LEDs in a section.
   *
   * @param section the section to modify
   * @param brightness the brightness out of 255
   * @return a command to set the brightness
   */
  public static byte[] setLEDSectionBrightness(int section, int brightness) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.Section,
      (byte) section,
      ControllerDataProtocol.LightingValue.LedBrightness,
      (byte) brightness
    };
  }

  /**
   * Set the brightness of every LED.
   *
   * @param brightness the brightness out of 255
   * @return a command to set the brightness
   */
  public static byte[] setAllLEDBrightness(int brightness) {
    return new byte[] {
      ControllerDataProtocol.CommandID.SetLed,
      ControllerDataProtocol.LightingSelection.All,
      ControllerDataProtocol.LightingValue.LedBrightness,
      (byte) brightness
    };
  }
}
