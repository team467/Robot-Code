package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class InvertedLEDStrip extends LEDStrip {

    protected InvertedLEDStrip(int length, int id) {
        super(length, id);
    }

    @Override
    public void setLED(int index, Color color) {
        super.setLED((super.getLength() - 1) - index, color);
    }

    @Override
    public void setLED(int index, Color8Bit color) {
        super.setLED((super.getLength() - 1) - index, color);
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        super.setRGB((super.getLength() - 1) - index, r, g, b);
    }

    @Override
    public void setHSV(int index, int h, int s, int v) {
        super.setHSV((super.getLength() - 1) - index, h, s, v);
    }

    @Override
    public void setHSB(int index, float h, float s, float b) {
        super.setHSB((super.getLength() - 1) - index, h, s, b);
    }

    @Override
    public void setHSB(int index, int h, int s, int b) {
        setHSB(index, h/360f, s/255f, b/255f);
    }
}
