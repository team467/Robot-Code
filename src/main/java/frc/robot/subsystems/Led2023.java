package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.leds.LEDManager;
import frc.lib.leds.LEDStrip;
import frc.robot.RobotConstants;

public class Led2023 extends SubsystemBase {
  public LEDStrip ledStrip;
  public static final boolean USE_BATTERY_CHECK = true;
  public static final double BATTER_MIN_VOLTAGE = 9.0;

  private final double SHOOTING_TIMER_SPEED = 0.1;
  private final double RAINBOW_TIMER_SPEED = 0.02;
  private final int RAINBOW_AMOUNT = 10;

  private double color = 0;
  private Timer rainbowTimer = new Timer();
  private Timer purpleTimer = new Timer();
  protected double lastLoopTime = 0;

  public static final double TARGET_MAX_RANGE = 100.0;
  public static final double TARGET_MAX_ANGLE = 15.0;
  public static final double BALL_MAX_RANGE = 100.0;
  public static final double BALL_MAX_ANGLE = 15.0;
  private COLORS_467 batteryCheckColor = COLORS_467.Orange;
  /*
   * Color blind preferred pallet includes White, Black, Red, Blue, Gold
   */
  public enum COLORS_467 {
    White(0xFF, 0xFF, 0xFF, 0xdc267f00),
    Red(0xFF, 0x00, 0x00, 0x99000000),
    Green(0x00, 0x80, 0x00, 0x33663300),
    Blue(0x00, 0x00, 0xCC, 0x1a339900),
    Gold(0xFF, 0xC2, 0x0A, 0xe6e64d00),
    Pink(0xDC, 0x26, 0x7F, 0xdc267f00),
    Orange(0xFE, 0x61, 0x00, 0xfe6100),
    Black(0x00, 0x00, 0x00, 0x00000000);

    public final int red;
    public final int green;
    public final int blue;
    public final int shuffleboard;

    COLORS_467(int red, int green, int blue, int shuffleboard) {
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.shuffleboard = shuffleboard;
    }

    public Color getColor() {
      return new Color(red, green, blue);
    }
  }

  public Led2023() {
    super();

    ledStrip = LEDManager.getInstance().createDoubleStrip(RobotConstants.get().led2023LedCount());
    for (int i = 0; i < ledStrip.getSize(); i++) {
      ledStrip.setRGB(i, 0, 0, 0);
    }
    rainbowTimer.start();
    purpleTimer.start();
  }

  public void resetTimers() {
    rainbowTimer.reset();
    purpleTimer.reset();
    lastLoopTime = Timer.getFPGATimestamp();
  }

  public void defaultLights() {
    if (USE_BATTERY_CHECK && RobotController.getBatteryVoltage() <= BATTER_MIN_VOLTAGE) {
      set(batteryCheckColor);
    } else {
      setRainbowMovingDown();
    }

    sendData();
    lastLoopTime = Timer.getFPGATimestamp();
  }

  public void setLED(int index, Color color) {
    ledStrip.setLED(index, color);
  }

  public void setLED(int index, Color8Bit color) {
    ledStrip.setLED(index, color);
  }

  public void setRGB(int index, int r, int g, int b) {
    ledStrip.setRGB(index, r, g, b);
  }

  public void setHSV(int index, int h, int s, int v) {
    ledStrip.setHSV(index, h, s, v);
  }

  public void setHSB(int index, float h, float s, float b) {
    ledStrip.setHSB(index, h, s, b);
  }

  public void setHSB(int index, int h, int s, int b) {
    setHSB(index, h / 360f, s / 255f, b / 255f);
  }

  public void sendData() {
    ledStrip.update();
  }

  public void set(Color color) {
    setTop(color);
    setBottom(color);
  }

  public void setTop(Color color) {
    for (int i = 0; i < RobotConstants.get().led2023LedCount() / 2; i++) {
      ledStrip.setLED(i, color);
    }
  }

  public void setBottom(Color color) {
    for (int i = RobotConstants.get().led2023LedCount() / 2;
        i < RobotConstants.get().led2023LedCount();
        i++) {
      ledStrip.setLED(i, color);
    }
  }

  public void set(COLORS_467 color) {
    setTop(color);
    setBottom(color);
  }

  public void setTop(COLORS_467 color) {
    for (int i = RobotConstants.get().led2023LedCount() / 2;
        i < RobotConstants.get().led2023LedCount();
        i++) {
      ledStrip.setRGB(i, color.red, color.green, color.blue);
    }
  }

  public void setBottom(COLORS_467 color) {
    for (int i = 0; i < RobotConstants.get().led2023LedCount() / 2; i++) {
      ledStrip.setRGB(i, color.red, color.green, color.blue);
    }
  }

  public void setColorMovingUp(Color fgColor, Color bgColor) {
    if (purpleTimer.hasElapsed(
        SHOOTING_TIMER_SPEED * (RobotConstants.get().led2023LedCount() + 2))) {
      purpleTimer.reset();
    }

    for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
      if (purpleTimer.hasElapsed(SHOOTING_TIMER_SPEED * i)) {
        double timeUntilOff = Math.max(0, (SHOOTING_TIMER_SPEED * (i + 2)) - purpleTimer.get());
        double brightness = (255 * timeUntilOff);

        if (brightness == 0) {
          ledStrip.setLED(i, bgColor);
        } else {
          ledStrip.setRGB(
              i,
              (int) (fgColor.red * brightness),
              (int) (fgColor.green * brightness),
              (int) (fgColor.blue * brightness));
        }
      } else {
        ledStrip.setLED(i, bgColor);
      }
    }
  }

  public void setColorMovingDown(Color fgColor, Color bgColor) {
    if (purpleTimer.hasElapsed(
        SHOOTING_TIMER_SPEED * (RobotConstants.get().led2023LedCount() + 2))) {
      purpleTimer.reset();
    }

    for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
      if (purpleTimer.hasElapsed(SHOOTING_TIMER_SPEED * i)) {
        double timeUntilOff = Math.max(0, (SHOOTING_TIMER_SPEED * (i + 2)) - purpleTimer.get());
        double brightness = (255 * timeUntilOff);

        if (brightness == 0) {
          ledStrip.setLED(i, bgColor);
        } else {
          ledStrip.setRGB(
              i,
              (int) (fgColor.red * brightness),
              (int) (fgColor.green * brightness),
              (int) (fgColor.blue * brightness));
        }
      } else {
        ledStrip.setLED(i, bgColor);
      }
    }
  }

  public void setRainbowMovingUp() {
    if (rainbowTimer.hasElapsed(RAINBOW_TIMER_SPEED)) {
      color -= RAINBOW_AMOUNT;

      if (color > 360) color = 0;
      rainbowTimer.reset();
    }

    for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
      ledStrip.setHSB(
          i, ((int) color + (i * 360 / RobotConstants.get().led2023LedCount())) % 360, 255, 127);
    }
  }

  public void setRainbowMovingDown() {
    if (rainbowTimer.hasElapsed(RAINBOW_TIMER_SPEED)) {
      color += RAINBOW_AMOUNT;

      if (color < 0) color = 360;
      rainbowTimer.reset();
    }

    for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
      ledStrip.setHSB(
          i, ((int) color + (i * 360 / RobotConstants.get().led2023LedCount())) % 360, 255, 127);
    }
  }

  public void setRainbow() {
    rainbowTimer.reset();
    for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
      ledStrip.setHSB(
          i, ((int) color + (i * 360 / RobotConstants.get().led2023LedCount())) % 360, 255, 127);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
