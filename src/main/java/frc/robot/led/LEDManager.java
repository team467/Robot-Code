package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.RobotConstants;

import java.util.ArrayList;
import java.util.HashMap;

public class LEDManager {
    private static LEDManager instance = null;
    private final ArrayList<Integer> offsets = new ArrayList<>();
    private int length = 0;
    private AddressableLEDBuffer ledBuffer;
    private AddressableLED ledStrip;

    public static LEDManager getInstance() {
        if (instance == null) instance = new LEDManager();
        return instance;
    }

    private LEDManager() {}

    public LEDStrip createStrip(int length) {
        offsets.add(length);
        this.length += length;

        return new LEDStrip(length, offsets.size() - 1);
    }

    public void init() {
        ledBuffer = new AddressableLEDBuffer(length);
        ledStrip = new AddressableLED(RobotConstants.get().ledChannel());
        ledStrip.setLength(length);
        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    public void update(AddressableLEDBuffer buffer, int offset) {
        for (int i = 0; i < buffer.getLength(); i++) {
            ledBuffer.setLED(i + offset, buffer.getLED(i));
        }
        ledStrip.setData(ledBuffer);
    }

    public void update(LEDStrip strip) {
        update(strip, offsets.get(strip.getId()));
    }
}
