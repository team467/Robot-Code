package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.leds.DoubleLEDStrip;
import frc.lib.leds.LEDManager;
import frc.robot.RobotConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;

public class Led2023 extends SubsystemBase {
  public DoubleLEDStrip ledStrip;
  public static final boolean USE_BATTERY_CHECK = true;
  public static final double BATTER_MIN_VOLTAGE = 10.0;
  public static final boolean CHECK_ARM_CALIBRATION = false;

  private final double SHOOTING_TIMER_SPEED = 0.1;
  private final double RAINBOW_TIMER_SPEED = 0.02;
  private final int RAINBOW_AMOUNT = 20;
  private IntakeRelease intakerelease;
  private Arm arm;

  private double color = 0;
  private Timer rainbowTimer = new Timer();
  private Timer purpleTimer = new Timer();
  protected double lastLoopTime = 0;
  private ColorScheme cmdColorScheme = ColorScheme.DEFAULT;

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
    Yellow(0xFF, 0xB1, 0x0A, 0xe6e69d00),
    Pink(0xDC, 0x26, 0x7F, 0xdc267f00),
    Orange(0xFE, 0x61, 0x00, 0xfe6100),
    Black(0x00, 0x00, 0x00, 0x00000000),
    Gold(0xFF, 0xC2, 0x0A, 0xe6e64d00),
    Purple(0x69, 0x03, 0xA3, 0x8000ff00);

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

  public enum ColorScheme {
    WANT_CUBE,
    WANT_CONE,
    HOLD_CUBE,
    HOLD_CONE,
    INTAKE_CUBE,
    INTAKE_CONE,
    RELEASE_CUBE,
    RELEASE_CONE,
    DEFAULT,
    BATTERY_LOW,
    ARM_UNCALIBRATED,
    CUBE_LOW,
    CUBE_MID,
    CUBE_HIGH,
    CONE_LOW,
    CONE_MID,
    CONE_HIGH,
    INTAKE_UNKNOWN,
    RELEASE_UNKNOWN,
    CALIBRATING,
    RESET_POSE,
    FLOOR,
    SHELF
  }

  public Led2023(Arm arm, IntakeRelease intakerelease) {
    super();
    this.arm = arm;
    this.intakerelease = intakerelease;
    ledStrip =
        LEDManager.getInstance().createDoubleStrip(RobotConstants.get().led2023LedCount(), false);
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

  public void setCmdColorScheme(ColorScheme cs) {
    cmdColorScheme = cs;
  }

  @Override
  public void periodic() {
    ColorScheme colorScheme = getColorScheme();
    switch (colorScheme) {
      case BATTERY_LOW:
        set(batteryCheckColor);
        sendData();
        break;
      case ARM_UNCALIBRATED:
        set(COLORS_467.Red);
        sendData();
        break;
      case CONE_HIGH:
        setOneThird(COLORS_467.Yellow, 3);
        sendData();
        break;
      case CONE_LOW:
        setOneThird(COLORS_467.Yellow, 1);
        sendData();
        break;
      case CONE_MID:
        setOneThird(COLORS_467.Yellow, 2);
        sendData();
        break;
      case CUBE_HIGH:
        setOneThird(COLORS_467.Purple, 3);
        sendData();
        break;
      case CUBE_LOW:
        setOneThird(COLORS_467.Purple, 1);
        sendData();
        break;
      case CUBE_MID:
        setOneThird(COLORS_467.Purple, 2);
        sendData();
        break;
      case DEFAULT:
        setRainbowMovingDownSecondInv();
        sendData();
        break;
      case HOLD_CONE:
        setBlinkColors(COLORS_467.Yellow, COLORS_467.Yellow, COLORS_467.White.getColor());
        sendData();
        break;
      case HOLD_CUBE:
        setBlinkColors(COLORS_467.Purple, COLORS_467.Purple, COLORS_467.White.getColor());
        sendData();
        break;
      case INTAKE_CONE:
        setColorMovingUp(COLORS_467.White.getColor(), COLORS_467.Yellow.getColor());
        sendData();
        break;
      case INTAKE_CUBE:
        setColorMovingUp(COLORS_467.White.getColor(), COLORS_467.Purple.getColor());
        sendData();
        break;
      case RELEASE_CONE:
        setColorMovingDown(COLORS_467.Black.getColor(), COLORS_467.Yellow.getColor());
        sendData();
        break;
      case RELEASE_CUBE:
        setColorMovingDown(COLORS_467.Black.getColor(), COLORS_467.Purple.getColor());
        break;
      case WANT_CONE:
        set(COLORS_467.Yellow);
        sendData();
        break;
      case WANT_CUBE:
        set(COLORS_467.Purple);
        sendData();
        break;
      case INTAKE_UNKNOWN:
        setColorMovingUpTwoClr(COLORS_467.Purple.getColor(), COLORS_467.Yellow.getColor());
        sendData();
        break;
      case RELEASE_UNKNOWN:
        setColorMovingDownTwoClr(COLORS_467.Yellow.getColor(), COLORS_467.Purple.getColor());
        sendData();
        break;
      case CALIBRATING:
        setBlinkColors(COLORS_467.Red, COLORS_467.Red, COLORS_467.Black.getColor());
        sendData();
        break;
      case RESET_POSE:
        setBlinkColors(COLORS_467.Orange, COLORS_467.Pink, COLORS_467.Green.getColor());
        sendData();
      case SHELF:
        if (intakerelease.getWants() == Wants.CONE) {
          setTop(COLORS_467.Yellow);
        } else {
          setTop(COLORS_467.Purple);
        }
        sendData();
        break;
      case FLOOR:
        if (intakerelease.getWants() == Wants.CONE) {
          setBottom(COLORS_467.Yellow);
        } else {
          setBottom(COLORS_467.Purple);
        }
        sendData();
        break;
      default:
        setRainbowMovingDownSecondInv();
        sendData();
        break;
    }
  }

  private ColorScheme getColorScheme() {
    if (USE_BATTERY_CHECK && RobotController.getBatteryVoltage() <= BATTER_MIN_VOLTAGE) {
      return ColorScheme.BATTERY_LOW;

    } else if (CHECK_ARM_CALIBRATION && !arm.isCalibrated()) {
      return ColorScheme.ARM_UNCALIBRATED;
    } else {
      return cmdColorScheme;
    }
  }

  public ColorScheme defaultLights() {
    sendData();
    lastLoopTime = Timer.getFPGATimestamp();
    if (USE_BATTERY_CHECK && RobotController.getBatteryVoltage() <= BATTER_MIN_VOLTAGE) {
      return ColorScheme.BATTERY_LOW;
    } else if ((!arm.isCalibrated()) && CHECK_ARM_CALIBRATION) {
      return ColorScheme.ARM_UNCALIBRATED;
    } else {
      return ColorScheme.DEFAULT;
    }
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
      ledStrip.update();
    }
  }

  public void setBottom(Color color) {
    for (int i = RobotConstants.get().led2023LedCount() / 2;
        i < RobotConstants.get().led2023LedCount();
        i++) {
      ledStrip.setLED(i, color);
      ledStrip.update();
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
      ledStrip.update();
    }
  }

  public void setBottom(COLORS_467 color) {
    for (int i = 0; i < RobotConstants.get().led2023LedCount() / 2; i++) {
      ledStrip.setRGB(i, color.red, color.green, color.blue);
      ledStrip.update();
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
          ledStrip.update();
        } else {
          ledStrip.setRGB(
              i,
              (int) (fgColor.red * brightness),
              (int) (fgColor.green * brightness),
              (int) (fgColor.blue * brightness));
          ledStrip.update();
        }
      } else {
        ledStrip.setLED(i, bgColor);
        ledStrip.update();
      }
    }
  }

  public void setColorMovingUp(Color fgColor, Color bgColor) {
    if (purpleTimer.hasElapsed(
        SHOOTING_TIMER_SPEED * (RobotConstants.get().led2023LedCount() + 2))) {
      purpleTimer.reset();
    }

    for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
      int j = RobotConstants.get().led2023LedCount() - i - 1;
      if (purpleTimer.hasElapsed(SHOOTING_TIMER_SPEED * i)) {
        double timeUntilOff = Math.max(0, (SHOOTING_TIMER_SPEED * (i + 2)) - purpleTimer.get());
        double brightness = (255 * timeUntilOff);

        if (brightness == 0) {
          ledStrip.setLED(j, bgColor);
          ledStrip.update();
        } else {
          ledStrip.setRGB(
              j,
              (int) (fgColor.red * brightness),
              (int) (fgColor.green * brightness),
              (int) (fgColor.blue * brightness));
          ledStrip.update();
        }
      } else {
        ledStrip.setLED(j, bgColor);
        ledStrip.update();
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
      ledStrip.update();
    }
  }

  public void setRainbowMovingDown() {
    if (rainbowTimer.hasElapsed(RAINBOW_TIMER_SPEED)) {
      color += RAINBOW_AMOUNT;
      ledStrip.update();
      if (color < 0) color = 360;
      rainbowTimer.reset();
    }

    for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
      ledStrip.setHSB(
          i, ((int) color + (i * 360 / RobotConstants.get().led2023LedCount())) % 360, 255, 127);
      ledStrip.update();
    }
  }

  public void setRainbowMovingDownSecondInv() {
    if (rainbowTimer.hasElapsed(RAINBOW_TIMER_SPEED)) {
      color += RAINBOW_AMOUNT;
      ledStrip.update();
      if (color < 0) color = 360;
      rainbowTimer.reset();
    }

    for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
      ledStrip.setLeftHSB(
          i, ((int) color + (i * 360 / RobotConstants.get().led2023LedCount())) % 360, 255, 127);
      ledStrip.setRightHSB(
          i, ((int) color - (i * 360 / RobotConstants.get().led2023LedCount())) % 360, 255, 127);
      ledStrip.update();
    }
  }

  public void setRainbow() {
    rainbowTimer.reset();
    for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
      ledStrip.setHSB(
          i, ((int) color + (i * 360 / RobotConstants.get().led2023LedCount())) % 360, 255, 127);
      ledStrip.update();
    }
  }

  public void setColorMovingDownTwoClr(Color topColor, Color bottomColor) {
    if (purpleTimer.hasElapsed(
        SHOOTING_TIMER_SPEED * (RobotConstants.get().led2023LedCount() + 2))) {
      purpleTimer.reset();
    }

    for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
      if (purpleTimer.hasElapsed(SHOOTING_TIMER_SPEED * i)) {
        double timeUntilOff = Math.max(0, (SHOOTING_TIMER_SPEED * (i + 2)) - purpleTimer.get());
        double brightness = (255 * timeUntilOff);

        if (brightness == 0) {
          if (i < RobotConstants.get().led2023LedCount() / 2) {
            ledStrip.setLED(i, topColor);
          } else {
            ledStrip.setLED(i, bottomColor);
          }
          ledStrip.update();
        } else {
          if (i < RobotConstants.get().led2023LedCount() / 2) {
            ledStrip.setRGB(
                i,
                (int) (topColor.red * brightness),
                (int) (topColor.green * brightness),
                (int) (topColor.blue * brightness));
          } else {
            ledStrip.setRGB(
                i,
                (int) (bottomColor.red * brightness),
                (int) (bottomColor.green * brightness),
                (int) (bottomColor.blue * brightness));
          }
          ledStrip.update();
        }
      } else {
        if (i < RobotConstants.get().led2023LedCount() / 2) {
          ledStrip.setLED(i, topColor);
        } else {
          ledStrip.setLED(i, bottomColor);
        }
        ledStrip.update();
      }
    }
  }

  public void setColorMovingUpTwoClr(Color topColor, Color bottomColor) {
    if (purpleTimer.hasElapsed(
        SHOOTING_TIMER_SPEED * (RobotConstants.get().led2023LedCount() + 2))) {
      purpleTimer.reset();
    }

    for (int i = RobotConstants.get().led2023LedCount() - 1; i >= 0; i--) {
      int j = RobotConstants.get().led2023LedCount() - 1 - i;
      if (purpleTimer.hasElapsed(SHOOTING_TIMER_SPEED * i)) {
        double timeUntilOff = Math.max(0, (SHOOTING_TIMER_SPEED * (i + 2)) - purpleTimer.get());
        double brightness = (255 * timeUntilOff);
        Color currentColor =
            j >= RobotConstants.get().led2023LedCount() / 2 ? topColor : bottomColor;

        if (brightness == 0) {
          ledStrip.setLED(j, currentColor);
          ledStrip.update();
        } else {
          ledStrip.setRGB(
              j,
              (int) (currentColor.red * brightness),
              (int) (currentColor.green * brightness),
              (int) (currentColor.blue * brightness));
          ledStrip.update();
        }
      } else {
        Color currentColor =
            j >= RobotConstants.get().led2023LedCount() / 2 ? topColor : bottomColor;
        ledStrip.setLED(j, currentColor);
        ledStrip.update();
      }
    }
  }

  public void setBlinkColors(COLORS_467 topColor, COLORS_467 bottomColor, Color bgColor) {

    if (purpleTimer.hasElapsed(0.6)) {
      purpleTimer.reset();
    } else if (purpleTimer.hasElapsed(0.25)) {
      setTop(topColor);
      setBottom(bottomColor);
      ledStrip.update();
    } else {
      set(bgColor);
      ledStrip.update();
    }
  }

  public void setAlternateColorsDown(COLORS_467 colorOne, COLORS_467 colorTwo, Color bgColor) {
    for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
      if (i % 2 == 0) {
        ledStrip.setLED(i, colorOne.getColor());
      } else {
        ledStrip.setLED(i, colorTwo.getColor());
      }
    }
    ledStrip.update();
    if (purpleTimer.hasElapsed(
        SHOOTING_TIMER_SPEED * (RobotConstants.get().led2023LedCount() + 2))) {
      purpleTimer.reset();
      for (int j = 0; j < RobotConstants.get().led2023LedCount(); j++) {
        ledStrip.setLED(j, bgColor);
        ledStrip.update();
      }
    }
  }

  public void setAlternateColorsUp(COLORS_467 colorOne, COLORS_467 colorTwo, Color bgColor) {
    for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
      if (i % 2 == 0) {
        ledStrip.setLED(i, colorOne.getColor());
      } else {
        ledStrip.setLED(i, colorTwo.getColor());
      }
    }
    ledStrip.update();
    for (int j = 0; j < RobotConstants.get().led2023LedCount(); j++) {
      int l = RobotConstants.get().led2023LedCount() - 1 - j;
      ledStrip.setLED(l, bgColor);
      ledStrip.update();
    }
  }

  public void setOneThird(COLORS_467 color, int preSet) {
    // t=1, 2, or 3. sets top 1/3, mid 1/3, or lower 1/3
    double start;
    double end;
    if (preSet == 1) {
      start = 0;
      end = Math.ceil(RobotConstants.get().led2023LedCount() / 3);
    } else if (preSet == 2) {
      start = Math.floor(RobotConstants.get().led2023LedCount() / 3);
      end =
          Math.ceil(
              RobotConstants.get().led2023LedCount()
                  - (RobotConstants.get().led2023LedCount() / 3));
    } else if (preSet == 3) {
      start =
          Math.floor(
              RobotConstants.get().led2023LedCount()
                  - (RobotConstants.get().led2023LedCount() / 3));
      end = Math.ceil(RobotConstants.get().led2023LedCount());
    } else {
      start = 0;
      end = RobotConstants.get().led2023LedCount();
    }
    for (int i = (int) start; i < end; i++) {
      ledStrip.setLED(i, color.getColor());
    }
    ledStrip.update();
  }
}
