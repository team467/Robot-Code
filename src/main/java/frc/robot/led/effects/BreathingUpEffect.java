package frc.robot.led.effects;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.led.LedUtil;

public class BreathingUpEffect implements Effect {
    @Override
    public int updateLED(int led, int t, int speed, int value, Color baseColorA, Color baseColorB, int brightness,
            AddressableLEDBuffer buffer) {
        LedUtil.setColor(buffer, led, LedUtil.brightnessCorrectColor(baseColorA, (value & 0xFF)/255f), brightness/255f);

        if ((t % speed) == 0) {
            return value + 1;
        }
        return value;
    }
}
