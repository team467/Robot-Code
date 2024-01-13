package frc.lib.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.util.ArrayList;

@Deprecated
public class LEDManager {

  private static LEDManager INSTANCE = null;
  private final ArrayList<Integer> offsets = new ArrayList<>();
  private int length = 0;
  private AddressableLEDBuffer ledBuffer;
  private AddressableLED ledStrip;

  /** Constructor should not be used, use {@code LEDManager.getInstance()} to get object instead. */
  private LEDManager() {
    // no constructor needed
  }

  /**
   * Gets the instance of the LEDManager class.
   *
   * @return A singleton instance of the LEDManager class.
   */
  public static LEDManager getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new LEDManager();
    }
    return INSTANCE;
  }

  /**
   * Creates a new LED strip with a certain length.
   *
   * @param length TODO: explain what this means
   * @return A LED strip that can be controlled.
   */
  public LEDStrip createStrip(int length) {
    offsets.add(this.length);
    this.length += length;

    return new LEDStrip(length, offsets.size() - 1);
  }

  /**
   * Creates a single new LED strip with a certain length.
   *
   * @param length TODO: explain what this means
   * @return A LED strip that can be controlled.
   */
  public LEDStrip createSingleStrip(int length) {
    return createStrip(length);
  }

  /**
   * Creates a new inverted LED strip with a certain length.
   *
   * @param length TODO: explain what this means
   * @return An inverted LED strip that can be controlled.
   */
  public InvertedLEDStrip createInvertedStrip(int length) {
    offsets.add(this.length);
    this.length += length;

    return new InvertedLEDStrip(length, offsets.size() - 1);
  }

  /**
   * Creates a new double LED strip with a certain length.
   *
   * @param length TODO: explain what this means
   * @return A double LED strip that can be controlled.
   */
  public DoubleLEDStrip createDoubleStrip(int length, boolean secondInverted) {
    offsets.add(this.length);
    this.length += length * 2;

    return new DoubleLEDStrip(length, offsets.size() - 1, secondInverted);
  }

  /** Initializes the LED manager */
  public void init(int ledChannel) {
    ledBuffer = new AddressableLEDBuffer(length);
    ledStrip = new AddressableLED(ledChannel);
    ledStrip.setLength(length);
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  /**
   * Updates the LED manager
   *
   * @param buffer TODO: explain what this means
   * @param offset TODO: explain what this means
   */
  public void update(AddressableLEDBuffer buffer, int offset) {
    for (int i = 0; i < buffer.getLength(); i++) {
      ledBuffer.setLED(i + offset, buffer.getLED(i));
    }
    ledStrip.setData(ledBuffer);
  }

  /**
   * Updates an LED strip
   *
   * @param strip the LED strip to update
   */
  public void update(LEDStrip strip) {
    update(strip, offsets.get(strip.getId()));
  }
}
