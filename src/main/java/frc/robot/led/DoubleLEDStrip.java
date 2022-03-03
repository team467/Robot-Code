package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class DoubleLEDStrip extends LEDStrip {
    private final boolean secondInverted;

    protected DoubleLEDStrip(int length, int id, boolean secondInverted) {
        super(length * 2, id);
        this.secondInverted = secondInverted;
    }

    protected DoubleLEDStrip(int length, int id) {
        this(length, id, false);
    }

    @Override
    public int getSize() {
        return getLength() / 2;
    }

    public void setLeftLED(int index, Color color) {
        super.setLED(index, color);
    }

    public void setRightLED(int index, Color color) {
        super.setLED(secondInverted ? (super.getLength() - 1) - index : getLength() + index, color);
    }

    @Override
    public void setLED(int index, Color color) {
        setLeftLED(index, color);
        setRightLED(index, color);
    }

    public void setLeftLED(int index, Color8Bit color) {
        super.setLED(index, color);
    }

    public void setRightLED(int index, Color8Bit color) {
        super.setLED(secondInverted ? (super.getLength() - 1) - index : getLength() + index, color);
    }

    @Override
    public void setLED(int index, Color8Bit color) {
        setLeftLED(index, color);
        setRightLED(index, color);
    }

    public void setLeftRGB(int index, int r, int g, int b) {
        super.setRGB(index, r, g, b);
    }

    public void setRightRGB(int index, int r, int g, int b) {
        super.setRGB(secondInverted ? (super.getLength() - 1) - index : getLength() + index, r, g, b);
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        setLeftRGB(index, r, g, b);
        setRightRGB(index, r, g, b);
    }

    public void setLeftHSV(int index, int h, int s, int v) {
        super.setHSV(index, h, s, v);
    }

    public void setRightHSV(int index, int h, int s, int v) {
        super.setHSV(secondInverted ? (super.getLength() - 1) - index : getLength() + index, h, s, v);
    }

    @Override
    public void setHSV(int index, int h, int s, int v) {
        setLeftHSV(index, h, s, v);
        setRightHSV(index, h, s, v);
    }

    public void setLeftHSB(int index, float h, float s, float b) {
        super.setHSB(index, h, s, b);
    }

    public void setRightHSB(int index, float h, float s, float b) {
        super.setHSB(secondInverted ? (super.getLength() - 1) - index : getLength() + index, h, s, b);
    }

    @Override
    public void setHSB(int index, float h, float s, float b) {
        setLeftHSB(index, h, s, b);
        setRightHSB(index, h, s, b);
    }

    public void setLeftHSB(int index, int h, int s, int b) {
        setLeftHSB(index, h/360f, s/255f, b/255f);
    }

    public void setRightHSB(int index, int h, int s, int b) {
        setRightHSB(index, h/360f, s/255f, b/255f);
    }

    @Override
    public void setHSB(int index, int h, int s, int b) {
        setHSB(index, h/360f, s/255f, b/255f);
    }
}
