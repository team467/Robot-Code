package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDStrip extends AddressableLEDBuffer {
    private final int id;

    protected LEDStrip(int length, int id) {
        super(length);
        this.id = id;
    }

    public int getSize() {
        return getLength();
    }

    public int getId() {
        return id;
    }

    public void update() {
        LEDManager.getInstance().update(this);
    }

    public void setHSB(int index, float h, float s, float b) {
        java.awt.Color outColor = java.awt.Color.getHSBColor(h, s, b);
        super.setRGB(index, outColor.getRed(), outColor.getGreen(), outColor.getBlue());
    }

    public void setHSB(int index, int h, int s, int b) {
        setHSB(index, h/360f, s/255f, b/255f);
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
    super.setRGB(index, r, g, b);
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
      super.setRGB(index, v, v, v);
      return;
    }

    final int region = h / 30;
    final int remainder = (h - (region * 30)) * 6;

    final int p = (v * (255 - s)) >> 8;
    final int q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    final int t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
      case 0:
        super.setRGB(index, v, t, p);
        break;
      case 1:
        super.setRGB(index, q, v, p);
        break;
      case 2:
        super.setRGB(index, p, v, t);
        break;
      case 3:
        super.setRGB(index, p, q, v);
        break;
      case 4:
        super.setRGB(index, t, p, v);
        break;
      default:
        super.setRGB(index, v, p, q);
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
    super.setRGB(index, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }

  /**
   * Sets a specific LED in the buffer.
   *
   * @param index The index to write
   * @param color The color of the LED
   */
  public void setLED(int index, Color8Bit color) {
    super.setRGB(index, color.red, color.green, color.blue);
  }
}
