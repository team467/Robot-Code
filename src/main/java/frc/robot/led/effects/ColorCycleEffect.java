package frc.robot.led.effects;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.led.LedUtil;

public class ColorCycleEffect implements Effect {
    @Override
    public int updateLED(int led, int t, int speed, int value, Color baseColorA, Color baseColorB, int brightness,
            AddressableLEDBuffer buffer) {
        double h = (double) (value & 0xFF) / 255;
        Color color = LedUtil.hslToRGB(h, 1, 0.5);
        LedUtil.setColor(buffer, led, color, brightness/255f);

        if ((t % speed) == 0) {
            return value + 1;
        }
        return value;
    }
}