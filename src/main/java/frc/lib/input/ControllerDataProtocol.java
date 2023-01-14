package frc.lib.input;

/** A human-readable protocol for running commands in the controller. */
public class ControllerDataProtocol {

  private ControllerDataProtocol() {}

  /** The command to run. */
  public static class CommandID {

    private CommandID() {}

    public static final byte GetProtocolVersion = 0x01;
    public static final byte GetTeamNumber = 0x02;
    public static final byte GetControllerState = 0x03;
    public static final byte GetLedData = 0x04;
    public static final byte GetLed = 0x05;
    public static final byte SetLed = 0x06;

    // ...
    public static final byte GetPortName = (byte) 0xFD;
    public static final byte EnterBootloader = (byte) 0xFE;
    public static final byte Error = (byte) 0xFF;
  }

  public static class LedData {

    private LedData() {}

    public static final byte LedCount = 0x01;
    public static final byte SectionCount = 0x02;
  }

  /** How many lights to select. */
  public static class LightingSelection {

    private LightingSelection() {}

    public static final byte Single = 0x01;
    public static final byte Multiple = 0x02;
    public static final byte Section = 0x03;
    public static final byte All = 0x04;
  }

  /** The value to modify. */
  public static class LightingValue {

    private LightingValue() {}

    public static final byte LedBaseColor = 0x01;
    public static final byte LedEffect = 0x02;
    public static final byte LedEffectSpaced = 0x03;
    public static final byte LedOffset = 0x04;
    public static final byte LedSpeed = 0x05;
    public static final byte LedBrightness = 0x06;
  }

  /** The effect to set the lights to. */
  public static class LightingEffect {

    private LightingEffect() {}

    public static final byte Static = 0x00;
    public static final byte BreathingUp = 0x01;
    public static final byte BreathingDown = 0x02;
    public static final byte ColorCycle = 0x03;
  }
}
