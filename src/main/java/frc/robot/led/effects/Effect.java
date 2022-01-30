package frc.robot.led.effects;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public interface Effect {
    public int updateLED(int led, int t, int speed, int value, Color baseColorA, Color baseColorB, int brightness, AddressableLEDBuffer buffer);
}
