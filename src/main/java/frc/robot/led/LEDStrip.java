package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDStrip extends AddressableLEDBuffer {
    private final int id;

    protected LEDStrip(int length, int id) {
        super(length);
        this.id = id;
    }

    public int getId() {
        return id;
    }

    public void update() {
        System.out.println("update strip");
        LEDManager.getInstance().update(this);
    }

    public void setHSB(int index, float h, float s, float b) {
        java.awt.Color outColor = java.awt.Color.getHSBColor(h, s, b);
        setRGB(index, outColor.getRed(), outColor.getGreen(), outColor.getBlue());
    }

    public void setHSB(int index, int h, int s, int b) {
        setHSB(index, h/360f, s/255f, b/255f);
    }
}
