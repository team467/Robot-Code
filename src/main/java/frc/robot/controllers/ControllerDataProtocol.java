package frc.robot.controllers;

public class ControllerDataProtocol {
    public static class CommandID {
        public static byte GetProtocolVersion = 0x01;
        public static byte GetTeamNumber = 0x02;
        public static byte GetControllerState = 0x03;
        public static byte GetLedData = 0x04;
        public static byte GetLed = 0x05;
        public static byte SetLed = 0x06;
    
    
        //...
        public static byte GetPortName = (byte) 0xFD;
        public static byte EnterBootloader = (byte) 0xFE;
        public static byte Error = (byte) 0xFF;
    }
    
    public static class LedData {
        public static byte LedCount = 0x01;
        public static byte SectionCount = 0x02;
    }
    
    public static class LightingSelection {
        public static byte Single = 0x01;
        public static byte Multiple = 0x02;
        public static byte Section = 0x03;
        public static byte All = 0x04;
    }
    
    public static class LightingValue {
        public static byte LedBaseColor = 0x01;
        public static byte LedEffect = 0x02;
        public static byte LedEffectSpaced = 0x03;
        public static byte LedOffset = 0x04;
        public static byte LedSpeed = 0x05;
        public static byte LedBrightness = 0x06;
    }

    public static class LightingEffect {
        public static byte Static = 0x00;
        public static byte BreathingUp = 0x01;
        public static byte BreathingDown = 0x02;
        public static byte ColorCycle = 0x03;
    }
}
