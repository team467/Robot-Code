package frc.lib.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Controls an LED Strip's color. */
@Deprecated
public class LEDStrip extends AddressableLEDBuffer {

  private static final int[] gammaTable = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
    5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14,
    15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27, 27,
    28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36, 37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47,
    48, 49, 50, 50, 51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68, 69, 70, 72, 73,
    74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89, 90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105,
    107, 109, 110, 112, 114, 115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138,
    140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175, 177,
    180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213, 215, 218, 220, 223,
    225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255
  };

  private final int id;

  protected LEDStrip(int length, int id) {
    super(length);
    this.id = id;
  }

  /**
   * Gets the size of the LED strip
   *
   * @return the length of the LED strip
   */
  public int getSize() {
    return getLength();
  }

  /**
   * Gets the ID of the LED strip
   *
   * @return the ID of the LED strip
   */
  public int getId() {
    return id;
  }

  /** Updates the LED strip */
  public void update() {
    LEDManager.getInstance().update(this);
  }

  /**
   * Sets a specific led in the buffer.
   *
   * @param index The index of the color you want to set.
   * @param h Hue, a value between 0 and 1.
   * @param s saturation
   * @param b brightness
   */
  public void setHSB(int index, float h, float s, float b) {
    java.awt.Color outColor = java.awt.Color.getHSBColor(h, s, b);
    setRGBGamma(index, outColor.getRed(), outColor.getGreen(), outColor.getBlue());
  }

  /**
   * Sets a specific led in the buffer.
   *
   * @param index the index to write
   * @param h Hue, a value between 0 and 1.
   * @param s saturation
   * @param b brightness
   */
  public void setHSB(int index, int h, int s, int b) {
    setHSB(index, h / 360f, s / 255f, b / 255f);
  }

  /**
   * Sets a specific led in the buffer.
   *
   * @param index the index to write
   * @param r the r value [0-255]
   * @param g the g value [0-255]
   * @param b the b value [0-255]
   */
  @SuppressWarnings("ParameterName")
  public void setRGBGamma(int index, int r, int g, int b) {
    super.setRGB(
        index,
        gammaTable[MathUtil.clamp(r, 0, 255)],
        gammaTable[MathUtil.clamp(g, 0, 255)],
        gammaTable[MathUtil.clamp(b, 0, 255)]);
  }

  /**
   * Sets a specific led in the buffer.
   *
   * @param index the index to write
   * @param r the r value [0-255]
   * @param g the g value [0-255]
   * @param b the b value [0-255]
   */
  @SuppressWarnings("ParameterName")
  public void setRGB(int index, int r, int g, int b) {
    setRGBGamma(index, r, g, b);
  }

  /**
   * Sets a specific led in the buffer.
   *
   * @param index the index to write
   * @param h the h value [0-180]
   * @param s the s value [0-255]
   * @param v the v value [0-255]
   */
  @SuppressWarnings("ParameterName")
  public void setHSV(final int index, final int h, final int s, final int v) {
    if (s == 0) {
      setRGBGamma(index, v, v, v);
      return;
    }

    final int region = h / 30;
    final int remainder = (h - (region * 30)) * 6;

    final int p = (v * (255 - s)) >> 8;
    final int q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    final int t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
      case 0:
        setRGBGamma(index, v, t, p);
        break;
      case 1:
        setRGBGamma(index, q, v, p);
        break;
      case 2:
        setRGBGamma(index, p, v, t);
        break;
      case 3:
        setRGBGamma(index, p, q, v);
        break;
      case 4:
        setRGBGamma(index, t, p, v);
        break;
      default:
        setRGBGamma(index, v, p, q);
        break;
    }
  }

  /**
   * Sets a specific LED in the buffer.
   *
   * @param index The index to write
   * @param color The color of the LED
   */
  public void setLED(int index, Color color) {
    setRGBGamma(
        index, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }

  /**
   * Sets a specific LED in the buffer.
   *
   * @param index The index to write
   * @param color The color of the LED
   */
  public void setLED(int index, Color8Bit color) {
    setRGBGamma(index, color.red, color.green, color.blue);
  }
}
