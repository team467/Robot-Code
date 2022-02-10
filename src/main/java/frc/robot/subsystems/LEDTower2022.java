package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.led.LEDManager;
import frc.robot.led.LEDStrip;

public class LEDTower2022 extends SubsystemBase{
    public LEDStrip ledStrip;


    public LEDTower2022() {
        super();

        ledStrip = LEDManager.getInstance().createStrip(RobotConstants.get().ledTower2022LEDCount() * 2);

        for (int i = 0; i < ledStrip.getLength(); i++) {
            ledStrip.setRGB(i, 0, 0, 0);
        }
    }

    public void setLeftLED(int index, Color color) {
        ledStrip.setLED(index, color);
    }

    public void setRightLED(int index, Color color) {
        ledStrip.setLED((ledStrip.getLength() - 1) - index, color);
    }

    public void setLED(int index, Color color) {
        setLeftLED(index, color);
        setRightLED(index, color);
    }

    public void setLeftLED(int index, Color8Bit color) {
        ledStrip.setLED(index, color);
    }

    public void setRightLED(int index, Color8Bit color) {
        ledStrip.setLED((ledStrip.getLength() - 1) - index, color);
    }

    public void setLED(int index, Color8Bit color) {
        setLeftLED(index, color);
        setRightLED(index, color);
    }

    public void setLeftRGB(int index, int r, int g, int b) {
        ledStrip.setRGB(index, r, g, b);
    }

    public void setRightRGB(int index, int r, int g, int b) {
        ledStrip.setRGB((ledStrip.getLength() - 1) - index, r, g, b);
    }

    public void setRGB(int index, int r, int g, int b) {
        setLeftRGB(index, r, g, b);
        setRightRGB(index, r, g, b);
    }

    public void setLeftHSV(int index, int h, int s, int v) {
        ledStrip.setHSV(index, h, s, v);
    }

    public void setRightHSV(int index, int h, int s, int v) {
        ledStrip.setHSV((ledStrip.getLength() - 1) - index, h, s, v);
    }

    public void setHSV(int index, int h, int s, int v) {
        setLeftHSV(index, h, s, v);
        setRightHSV(index, h, s, v);
    }

    public void setLeftHSB(int index, float h, float s, float b) {
        ledStrip.setHSB(index, h, s, b);
    }

    public void setRightHSB(int index, float h, float s, float b) {
        ledStrip.setHSB((ledStrip.getLength() - 1) - index, h, s, b);
    }

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

    public void setHSB(int index, int h, int s, int b) {
        setHSB(index, h/360f, s/255f, b/255f);
    }

    public void sendData() {
        ledStrip.update();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}


