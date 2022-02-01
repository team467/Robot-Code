package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class LEDTower2022 extends SubsystemBase{
    public enum Status {
        OFF,
        FOUND_BALL,
        CHASING_BALL,
        TEAM467,
        COLOR_CYCLE,
        RAINBOW
    };

    public AddressableLEDBuffer ledBuffer;
    public AddressableLED ledStrip;


    public LEDTower2022() {
        super();

        ledBuffer = new AddressableLEDBuffer(RobotConstants.get().ledTower2022LEDCount() * 2);
        ledStrip = new AddressableLED(RobotConstants.get().ledTower2022LEDChannel());
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();

        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
    }

    public void setLeftLED(int index, Color color) {
        ledBuffer.setLED(index, color);
    }

    public void setRightLED(int index, Color color) {
        ledBuffer.setLED((ledBuffer.getLength() - 1) - index, color);
    }

    public void setLED(int index, Color color) {
        setLeftLED(index, color);
        setRightLED(index, color);
    }

    public void setLeftLED(int index, Color8Bit color) {
        ledBuffer.setLED(index, color);
    }

    public void setRightLED(int index, Color8Bit color) {
        ledBuffer.setLED((ledBuffer.getLength() - 1) - index, color);
    }

    public void setLED(int index, Color8Bit color) {
        setLeftLED(index, color);
        setRightLED(index, color);
    }

    public void setLeftRGB(int index, int r, int g, int b) {
        ledBuffer.setRGB(index, r, g, b);
    }

    public void setRightRGB(int index, int r, int g, int b) {
        ledBuffer.setRGB((ledBuffer.getLength() - 1) - index, r, g, b);
    }

    public void setRGB(int index, int r, int g, int b) {
        setLeftRGB(index, r, g, b);
        setRightRGB(index, r, g, b);
    }

    public void setLeftHSV(int index, int h, int s, int v) {
        ledBuffer.setHSV(index, h, s, v);
    }

    public void setRightHSV(int index, int h, int s, int v) {
        ledBuffer.setHSV((ledBuffer.getLength() - 1) - index, h, s, v);
    }

    public void setHSV(int index, int h, int s, int v) {
        setLeftHSV(index, h, s, v);
        setRightHSV(index, h, s, v);
    }

    public void setLeftHSB(int index, float h, float s, float b) {
        java.awt.Color outColor = java.awt.Color.getHSBColor(h, s, b);
        ledBuffer.setRGB(index, outColor.getRed(), outColor.getGreen(), outColor.getBlue());
    }

    public void setRightHSB(int index, float h, float s, float b) {
        java.awt.Color outColor = java.awt.Color.getHSBColor(h, s, b);
        ledBuffer.setRGB((ledBuffer.getLength() - 1) - index, outColor.getRed(), outColor.getGreen(), outColor.getBlue());
    }

    public void setHSB(int index, float h, float s, float b) {
        setLeftHSB(index, h, s, b);
        setRightHSB(index, h, s, b);
        System.out.println(String.format("leftIndex: %d, rightIndex: %d", index, (ledBuffer.getLength() - 1) - index));
    }

    public void setLeftHSB(int index, int h, int s, int b) {
        setLeftHSB(index, h/360f, s/255f, b/255f);
    }

    public void setRightHSB(int index, int h, int s, int b) {
        setRightHSB(index, h/360f, s/255f, b/255f);
    }

    public void setHSB(int index, int h, int s, int b) {
        setHSB(index, h/360f, s/255f, b/255f);
    }

    public void sendData() {
        ledStrip.setData(ledBuffer);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}


