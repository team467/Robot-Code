package frc.robot.controllers;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ControllerCommandFactory {
    public static byte[] setLEDRGB(int index, int r, int g, int b) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Single, (byte) index, ControllerDataProtocol.LightingValue.LedBaseColor, (byte) r, (byte) g, (byte) b};
    }

    public static byte[] setLEDBaseColor(int index, Color8Bit color) {
        return setLEDRGB(index, color.red, color.green, color.blue);
    }

    public static byte[] setLEDBaseColor(int index, Color color) {
        return setLEDBaseColor(index, new Color8Bit(color));
    }

    public static byte[] setLEDHSB(int index, float h, float s, float b) {
        java.awt.Color outColor = java.awt.Color.getHSBColor(h, s, b);
        return setLEDRGB(index, outColor.getRed(), outColor.getGreen(), outColor.getBlue());
    }

    public static byte[] setLEDHSB(int index, int h, int s, int b) {
        return setLEDHSB(index, h/360f, s/255f, b/255f);
    }

    public static byte[] setLEDRangeRGB(int startLED, int endLED, int r, int g, int b) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Multiple, (byte) startLED, (byte) endLED, ControllerDataProtocol.LightingValue.LedBaseColor, (byte) r, (byte) g, (byte) b};
    }

    public static byte[] setLEDRangeBaseColor(int startLED, int endLED, Color8Bit color) {
        return setLEDRangeRGB(startLED, endLED, color.red, color.green, color.blue);
    }

    public static byte[] setLEDRangeBaseColor(int startLED, int endLED, Color color) {
        return setLEDRangeBaseColor(startLED, endLED, new Color8Bit(color));
    }

    public static byte[] setLEDRangeHSB(int startLED, int endLED, float h, float s, float b) {
        java.awt.Color outColor = java.awt.Color.getHSBColor(h, s, b);
        return setLEDRangeRGB(startLED, endLED, outColor.getRed(), outColor.getGreen(), outColor.getBlue());
    }

    public static byte[] setLEDRangeHSB(int startLED, int endLED, int h, int s, int b) {
        return setLEDRangeHSB(startLED, endLED, h/360f, s/255f, b/255f);
    }

    public static byte[] setLEDSectionRGB(int section, int r, int g, int b) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Section, (byte) section, ControllerDataProtocol.LightingValue.LedBaseColor, (byte) r, (byte) g, (byte) b};
    }

    public static byte[] setLEDSectionBaseColor(int section, Color8Bit color) {
        return setLEDSectionRGB(section, color.red, color.green, color.blue);
    }

    public static byte[] setLEDSectionBaseColor(int section, Color color) {
        return setLEDSectionBaseColor(section, new Color8Bit(color));
    }

    public static byte[] setLEDSectionHSB(int section, float h, float s, float b) {
        java.awt.Color outColor = java.awt.Color.getHSBColor(h, s, b);
        return setLEDSectionRGB(section, outColor.getRed(), outColor.getGreen(), outColor.getBlue());
    }

    public static byte[] setLEDSectionHSB(int section, int h, int s, int b) {
        return setLEDSectionHSB(section, h/360f, s/255f, b/255f);
    }

    public static byte[] setAllLEDRGB(int r, int g, int b) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.All, ControllerDataProtocol.LightingValue.LedBaseColor, (byte) r, (byte) g, (byte) b};
    }

    public static byte[] setAllLEDBaseColor(Color8Bit color) {
        return setAllLEDRGB(color.red, color.green, color.blue);
    }

    public static byte[] setAllLEDBaseColor(Color color) {
        return setAllLEDBaseColor(new Color8Bit(color));
    }

    public static byte[] setAllLEDHSB(float h, float s, float b) {
        java.awt.Color outColor = java.awt.Color.getHSBColor(h, s, b);
        return setAllLEDRGB(outColor.getRed(), outColor.getGreen(), outColor.getBlue());
    }

    public static byte[] setAllLEDHSB(int h, int s, int b) {
        return setAllLEDHSB(h/360f, s/255f, b/255f);
    }


    public static byte[] setLEDEffect(int index, int effect) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Single, (byte) index, ControllerDataProtocol.LightingValue.LedEffect, (byte) effect};
    }

    public static byte[] setLEDRangeEffect(int startLED, int endLED, int effect) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Multiple, (byte) startLED, (byte) endLED, ControllerDataProtocol.LightingValue.LedEffect, (byte) effect};
    }

    public static byte[] setLEDSectionEffect(int section, int effect) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Section, (byte) section, ControllerDataProtocol.LightingValue.LedEffect, (byte) effect};
    }

    public static byte[] setAllLEDEffect(int effect) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.All, ControllerDataProtocol.LightingValue.LedEffect, (byte) effect};
    }

    public static byte[] setLEDRangeEffectSpaced(int startLED, int endLED, int effect) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Multiple, (byte) startLED, (byte) endLED, ControllerDataProtocol.LightingValue.LedEffectSpaced, (byte) effect};
    }

    public static byte[] setLEDSectionEffectSpaced(int section, int effect) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Section, (byte) section, ControllerDataProtocol.LightingValue.LedEffectSpaced, (byte) effect};
    }

    public static byte[] setAllLEDEffectSpaced(int effect) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.All, ControllerDataProtocol.LightingValue.LedEffectSpaced, (byte) effect};
    }


    public static byte[] setLEDOffset(int index, int offset) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Single, (byte) index, ControllerDataProtocol.LightingValue.LedOffset, (byte) offset};
    }

    public static byte[] setLEDRangeOffset(int startLED, int endLED, int offset) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Multiple, (byte) startLED, (byte) endLED, ControllerDataProtocol.LightingValue.LedOffset, (byte) offset};
    }

    public static byte[] setLEDSectionOffset(int section, int offset) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Section, (byte) section, ControllerDataProtocol.LightingValue.LedOffset, (byte) offset};
    }

    public static byte[] setAllLEDOffset(int offset) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.All, ControllerDataProtocol.LightingValue.LedOffset, (byte) offset};
    }


    public static byte[] setLEDSpeed(int index, int speed) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Single, (byte) index, ControllerDataProtocol.LightingValue.LedSpeed, (byte) speed};
    }

    public static byte[] setLEDRangeSpeed(int startLED, int endLED, int speed) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Multiple, (byte) startLED, (byte) endLED, ControllerDataProtocol.LightingValue.LedSpeed, (byte) speed};
    }

    public static byte[] setLEDSectionSpeed(int section, int speed) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Section, (byte) section, ControllerDataProtocol.LightingValue.LedSpeed, (byte) speed};
    }

    public static byte[] setAllLEDSpeed(int speed) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.All, ControllerDataProtocol.LightingValue.LedSpeed, (byte) speed};
    }


    public static byte[] setLEDBrightness(int index, int brightness) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Single, (byte) index, ControllerDataProtocol.LightingValue.LedBrightness, (byte) brightness};
    }

    public static byte[] setLEDRangeBrightness(int startLED, int endLED, int brightness) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Multiple, (byte) startLED, (byte) endLED, ControllerDataProtocol.LightingValue.LedBrightness, (byte) brightness};
    }

    public static byte[] setLEDSectionBrightness(int section, int brightness) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.Section, (byte) section, ControllerDataProtocol.LightingValue.LedBrightness, (byte) brightness};
    }

    public static byte[] setAllLEDBrightness(int brightness) {
        return new byte[] {ControllerDataProtocol.CommandID.SetLed, ControllerDataProtocol.LightingSelection.All, ControllerDataProtocol.LightingValue.LedBrightness, (byte) brightness};
    }
}
