package frc.robot.led.effects;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.led.LedUtil;

public class StaticEffect implements Effect {

    @Override
    public int updateLED(int led, int t, int speed, int value, Color baseColorA, Color baseColorB, int brightness,
            AddressableLEDBuffer buffer) {
        LedUtil.setColor(buffer, led, baseColorA, brightness/255f);
        return 0;
    }

}
