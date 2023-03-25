package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.leds.DoubleLEDStrip;
import frc.lib.leds.LEDManager;
import frc.robot.RobotConstants;
import frc.robot.commands.arm.ArmCalibrateCMD;
import frc.robot.commands.arm.ArmFloorCMD;
import frc.robot.commands.arm.ArmScoreHighNodeCMD;
import frc.robot.commands.arm.ArmScoreMidNodeCMD;
import frc.robot.commands.arm.ArmShelfCMD;
import frc.robot.commands.auto.BetterBalancing;
import frc.robot.commands.intakerelease.HoldCMD;
import frc.robot.commands.intakerelease.IntakeAndRaise;
import frc.robot.commands.intakerelease.IntakeCMD;
import frc.robot.commands.intakerelease.ReleaseCMD;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class Led2023 extends SubsystemBase {
  public DoubleLEDStrip ledStrip;

  private Timer balanceTimer = new Timer();
  private Timer defaultTimer = new Timer();
  private ColorScheme lastColorScheme;
  private boolean finishedRainbowOnce = false;
  private boolean doneBalanceLeds = false;
  boolean balanceStarted = false;

  private static final boolean USE_BATTERY_CHECK = true;
  private static final double BATTER_MIN_VOLTAGE = 9.0;
  private static final boolean CHECK_ARM_CALIBRATION = true;
  private static final double RAINBOW_TIME_AFTER_CALIBRATION = 3;
  private static final COLORS_467 BATTERY_LOW_COLOR = COLORS_467.Orange;
  private static final COLORS_467 ARM_UNCALIBRATED_COLOR = COLORS_467.Red;

  private IntakeRelease intakerelease;
  private Arm arm;
  private Drive drive;

  private VictoryLeds balanceVictoryLeds = new VictoryLeds(COLORS_467.Blue, COLORS_467.Gold);
  private VictoryLeds scoreVictoryLeds = new VictoryLeds(COLORS_467.Yellow, COLORS_467.Purple);
  private Rainbows rainbowLed = new Rainbows();
  private Patterns colorPatterns = new Patterns();
  private SetThirdLeds setOneThird = new SetThirdLeds();

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
    SHELF,
    BALANCE_VICTORY,
    AUTO_SCORE
  }

  public Led2023(
      Arm arm,
      IntakeRelease intakerelease,
      Drive drive) {
    super();
    this.intakerelease = intakerelease;
    this.arm = arm;
    this.drive = drive;

    ledStrip =
        LEDManager.getInstance().createDoubleStrip(RobotConstants.get().led2023LedCount(), false);
    for (int i = 0; i < ledStrip.getSize(); i++) {
      ledStrip.setRGB(i, 0, 0, 0);
    }
    rainbowLed.rainbowTimer.start();
    colorPatterns.purpleTimer.start();
  }

  public void resetTimers() {
    rainbowLed.rainbowTimer.reset();
    colorPatterns.purpleTimer.reset();
    balanceTimer.reset();
  }

  @Override
  public void periodic() {
    ColorScheme colorScheme = getColorScheme();

    // Clears leds if colorSceme changed
    if (colorScheme != lastColorScheme) {
      set(COLORS_467.Black);
      lastColorScheme = colorScheme;
    }
    applyColorScheme(colorScheme);
    sendData();
  }

  public ColorScheme getColorScheme() {

    // Check if battery is low
    if (USE_BATTERY_CHECK && RobotController.getBatteryVoltage() <= BATTER_MIN_VOLTAGE) {
      return ColorScheme.BATTERY_LOW;
    }
    // Check if arm is calibrated
    if ((!arm.isCalibrated()) && CHECK_ARM_CALIBRATION && !DriverStation.isDisabled()) {
      return ColorScheme.ARM_UNCALIBRATED;
    }
    // When robot is disabled
    if (DriverStation.isDisabled()) {
      if (balanceTimer.get()>0.0&&!balanceTimer.hasElapsed(1)&&!doneBalanceLeds) {
        return ColorScheme.BALANCE_VICTORY;
      }
      defaultTimer.stop();
      defaultTimer.reset();
      balanceTimer.reset();
      doneBalanceLeds = true;
      balanceStarted = false;
      return ColorScheme.DEFAULT;
    }

    
    if (DriverStation.isAutonomousEnabled()) {
      // When robot is balanced in Autonomous
      if (drive.getCurrentCommand() instanceof BetterBalancing) {
        balanceStarted = true;
      }
      if (drive.isUpright()&&balanceStarted) {
        doneBalanceLeds = false;
        balanceTimer.start();
        return ColorScheme.BALANCE_VICTORY;
      }
      // When robot scores in autonomous
      if (intakerelease.getCurrentCommand() instanceof ReleaseCMD
          || intakerelease.getCurrentCommand() instanceof IntakeCMD
          || intakerelease.getCurrentCommand() instanceof IntakeAndRaise) {
        return ColorScheme.AUTO_SCORE;
      }
    }

    // When the arm is calibrating
    if (arm.getCurrentCommand() instanceof ArmCalibrateCMD) {
      return ColorScheme.CALIBRATING;
    }

    // Sets rainbow for 5 secs after calibrating
    if ((arm.isCalibrated() || !CHECK_ARM_CALIBRATION)
        && !defaultTimer.hasElapsed(RAINBOW_TIME_AFTER_CALIBRATION + 0.02)
        && DriverStation.isTeleopEnabled()
        && !finishedRainbowOnce) {
      if (defaultTimer.hasElapsed(RAINBOW_TIME_AFTER_CALIBRATION) && !finishedRainbowOnce) {
        defaultTimer.reset();
        defaultTimer.stop();
        finishedRainbowOnce = true;
      }
      defaultTimer.start();
      return ColorScheme.DEFAULT;
    }

    // If trying to hold on to something
    if (intakerelease.getCurrentCommand() instanceof HoldCMD) {
      // If holding on to Cubes
      if (intakerelease.haveCube() && !intakerelease.haveCone()) {
        return ColorScheme.HOLD_CUBE;
      }
      // If holding on to Cones
      if (intakerelease.haveCone()) {
        return ColorScheme.HOLD_CONE;
      }
    }
    // When picking up from shelf
    if (arm.getCurrentCommand() instanceof ArmShelfCMD) {
      return ColorScheme.SHELF;
    }

    // When picking up from floor
    if (arm.getCurrentCommand() instanceof ArmFloorCMD) {
      return ColorScheme.FLOOR;
    }

    // When arm is scoring high
    if (arm.getCurrentCommand() instanceof ArmScoreHighNodeCMD) {
      if (intakerelease.wantsCube() || (intakerelease.haveCube() && !intakerelease.haveCone())) {
        return ColorScheme.CUBE_HIGH;
      } else {
        return ColorScheme.CONE_HIGH;
      }
    }

    // When arm is scoring mid node
    if (arm.getCurrentCommand() instanceof ArmScoreMidNodeCMD) {
      if (intakerelease.wantsCube() || (intakerelease.haveCube() && !intakerelease.haveCone())) {
        return ColorScheme.CUBE_HIGH;
      } else {
        return ColorScheme.CONE_HIGH;
      }
    }

    // When arm is scoring hybrid/low node
    if (arm.getCurrentCommand() instanceof ArmScoreHighNodeCMD) {
      if (intakerelease.wantsCube() || (intakerelease.haveCube() && !intakerelease.haveCone())) {
        return ColorScheme.CUBE_HIGH;
      } else {
        return ColorScheme.CONE_HIGH;
      }
    }

    // If trying to intake something
    if (intakerelease.getCurrentCommand() instanceof IntakeCMD
        || intakerelease.getCurrentCommand() instanceof IntakeAndRaise) {
      // If intaking Cubes
      if (intakerelease.wantsCube()) {
        return ColorScheme.INTAKE_CUBE;
      }
      // If intaking Cones
      if (intakerelease.wantsCone()) {
        return ColorScheme.INTAKE_CONE;
      } else {
        return ColorScheme.INTAKE_UNKNOWN;
      }
    }

    // If trying to release something
    if (intakerelease.getCurrentCommand() instanceof ReleaseCMD) {
      // If releasing Cubes
      if (intakerelease.wantsCube()) {
        return ColorScheme.RELEASE_CUBE;
      }
      // If releasing Cones
      if (intakerelease.wantsCone()) {
        return ColorScheme.RELEASE_CONE;
      } else {
        return ColorScheme.RELEASE_UNKNOWN;
      }
    }

    // If we want cubes
    if (intakerelease.wantsCube()) {
      return ColorScheme.WANT_CUBE;
    }

    // If we want cones
    if (intakerelease.wantsCone()) {
      return ColorScheme.WANT_CONE;
    }

    // Sets default (never used)
    return ColorScheme.DEFAULT;
  }

  public void applyColorScheme(ColorScheme colorScheme) {
    switch (colorScheme) {
      case BATTERY_LOW:
        set(BATTERY_LOW_COLOR);
        break;
      case ARM_UNCALIBRATED:
        set(ARM_UNCALIBRATED_COLOR);
        break;
      case CONE_HIGH:
        setOneThird.set(COLORS_467.Yellow, 3);
        break;
      case CONE_LOW:
        setOneThird.set(COLORS_467.Yellow, 1);
        break;
      case CONE_MID:
        setOneThird.set(COLORS_467.Yellow, 2);
        break;
      case CUBE_HIGH:
        setOneThird.set(COLORS_467.Purple, 3);
        break;
      case CUBE_LOW:
        setOneThird.set(COLORS_467.Purple, 1);
        break;
      case CUBE_MID:
        setOneThird.set(COLORS_467.Purple, 2);
        break;
      case DEFAULT:
        rainbowLed.setRainbowMovingDownSecondInv();
        break;
      case HOLD_CONE:
        colorPatterns.setBlinkColors(
            COLORS_467.Yellow, COLORS_467.Yellow, COLORS_467.White.getColor());
        break;
      case HOLD_CUBE:
        colorPatterns.setBlinkColors(
            COLORS_467.Purple, COLORS_467.Purple, COLORS_467.White.getColor());
        break;
      case INTAKE_CONE:
        colorPatterns.setColorMovingUp(COLORS_467.White.getColor(), COLORS_467.Yellow.getColor());
        break;
      case INTAKE_CUBE:
        colorPatterns.setColorMovingUp(COLORS_467.White.getColor(), COLORS_467.Purple.getColor());
        break;
      case RELEASE_CONE:
        colorPatterns.setColorMovingDown(COLORS_467.Black.getColor(), COLORS_467.Yellow.getColor());
        break;
      case RELEASE_CUBE:
        colorPatterns.setColorMovingDown(COLORS_467.Black.getColor(), COLORS_467.Purple.getColor());
        break;
      case WANT_CONE:
        set(COLORS_467.Yellow);
        break;
      case WANT_CUBE:
        set(COLORS_467.Purple);
        break;
      case INTAKE_UNKNOWN:
        colorPatterns.setColorMovingUpTwoClr(
            COLORS_467.Purple.getColor(), COLORS_467.Yellow.getColor());
        break;
      case RELEASE_UNKNOWN:
        colorPatterns.setColorMovingDownTwoClr(
            COLORS_467.Yellow.getColor(), COLORS_467.Purple.getColor());
        break;
      case CALIBRATING:
        colorPatterns.setBlinkColors(
            ARM_UNCALIBRATED_COLOR, ARM_UNCALIBRATED_COLOR, COLORS_467.Black.getColor());
        break;
      case RESET_POSE:
        colorPatterns.setBlinkColors(
            COLORS_467.Orange, COLORS_467.Pink, COLORS_467.Green.getColor());
        break;
      case SHELF:
        if (intakerelease.wantsCone()) {
          setTop(COLORS_467.Yellow);
          setBottom(COLORS_467.Black);
        } else {
          setTop(COLORS_467.Purple);
          setBottom(COLORS_467.Black);
        }
        break;
      case FLOOR:
        if (intakerelease.wantsCone()) {
          setBottom(COLORS_467.Yellow);
          setTop(COLORS_467.Black);
        } else {
          setBottom(COLORS_467.Purple);
          setTop(COLORS_467.Black);
        }
        break;
      case BALANCE_VICTORY:
        balanceVictoryLeds.periodic();
        break;
      case AUTO_SCORE:
        scoreVictoryLeds.periodic();
        break;
      default:
        rainbowLed.setRainbowMovingDownSecondInv();
        break;
    }
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

  public class Patterns {
    private Timer purpleTimer = new Timer();
    private final double SHOOTING_TIMER_SPEED = 0.1;

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
  }

  private class Rainbows {

    private final double RAINBOW_TIMER_SPEED = 0.04;
    private final int RAINBOW_AMOUNT = 10;
    private double rainbowColor = 0;
    private Timer rainbowTimer = new Timer();

    public void setRainbowMovingUp() {
      if (rainbowTimer.hasElapsed(RAINBOW_TIMER_SPEED)) {
        rainbowColor -= RAINBOW_AMOUNT;

        if (rainbowColor > 360) rainbowColor = 0;
        rainbowTimer.reset();
      }

      for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
        ledStrip.setHSB(
            i,
            ((int) rainbowColor + (i * 360 / RobotConstants.get().led2023LedCount())) % 360,
            255,
            127);
        ledStrip.update();
      }
    }

    public void setRainbowMovingDown() {
      if (rainbowTimer.hasElapsed(RAINBOW_TIMER_SPEED)) {
        rainbowColor += RAINBOW_AMOUNT;
        ledStrip.update();
        if (rainbowColor < 0) rainbowColor = 360;
        rainbowTimer.reset();
      }

      for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
        ledStrip.setHSB(
            i,
            ((int) rainbowColor + (i * 360 / RobotConstants.get().led2023LedCount())) % 360,
            255,
            127);
        ledStrip.update();
      }
    }

    public void setRainbowMovingDownSecondInv() {
      if (rainbowTimer.hasElapsed(RAINBOW_TIMER_SPEED)) {
        rainbowColor += RAINBOW_AMOUNT;
        ledStrip.update();
        if (rainbowColor < 0) rainbowColor = 360;
        rainbowTimer.reset();
      }

      for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
        ledStrip.setLeftHSB(
            i,
            ((int) rainbowColor + (i * 360 / RobotConstants.get().led2023LedCount())) % 360,
            255,
            127);
        ledStrip.setRightHSB(
            i,
            ((int) rainbowColor - (i * 360 / RobotConstants.get().led2023LedCount())) % 360,
            255,
            127);
        ledStrip.update();
      }
    }

    public void setRainbow() {
      rainbowTimer.reset();
      for (int i = 0; i < RobotConstants.get().led2023LedCount(); i++) {
        ledStrip.setHSB(
            i,
            ((int) rainbowColor + (i * 360 / RobotConstants.get().led2023LedCount())) % 360,
            255,
            127);
        ledStrip.update();
      }
    }
  }

  public class SetThirdLeds {
    private static final int topStart = 0;
    private static final int topEndandMidStart = (int) (RobotConstants.get().led2023LedCount() / 3);
    private static final int midEndandBottomStart =
        (int)
            (RobotConstants.get().led2023LedCount() - (RobotConstants.get().led2023LedCount() / 3));
    private static final int bottomEnd = (RobotConstants.get().led2023LedCount());

    public void set(COLORS_467 color, int preSet) {
      // t=1, 2, or 3. sets top 1/3, mid 1/3, or lower 1/3
      int start;
      int end;

      if (preSet == 1) {
        start = topStart;
        end = topEndandMidStart + 1;
      } else if (preSet == 2) {
        start = topEndandMidStart;
        end = midEndandBottomStart;

      } else {
        start = midEndandBottomStart - 1;
        end = bottomEnd;
      }
      for (int i = start; i < end; i++) {
        ledStrip.setLED(i, color.getColor());
      }

      ledStrip.update();
    }
  }

  private class VictoryLeds {
    private COLORS_467 topClr;
    private COLORS_467 bottomClr;
    private boolean bright = false;
    private int brightness = 5;
    private static final int FADE_DURATION = 30;
    private static int fadeToWhite = 0;

    COLORS_467 fgColor;
    COLORS_467 bgColor;

    VictoryLeds(COLORS_467 fgColor, COLORS_467 bgColor) {
      this.fgColor = fgColor;
      this.bgColor = bgColor;
    }

    public void periodic() {
      if (topClr == null) {
        topClr = fgColor;
      }
      if (bottomClr == null) {
        bottomClr = bgColor;
      }
      if (brightness >= FADE_DURATION * 1.3 || bright) {
        if (brightness >= FADE_DURATION * 1.3) {
          if (topClr == fgColor) {
            topClr = bgColor;
            bottomClr = fgColor;
          } else {
            topClr = fgColor;
            bottomClr = bgColor;
          }
        }
        brightness = brightness - 2;
        bright = true;
      }
      if (brightness <= 5 || !bright) {
        brightness = brightness + 2;
        bright = false;
      }
      if (brightness > FADE_DURATION) {
        fadeToWhite = (brightness - FADE_DURATION) * 5;
      }
      for (int i = 0; i < RobotConstants.get().led2023LedCount() / 2; i++) {

        ledStrip.setRGB(
            i,
            Math.min((int) (topClr.red * brightness / FADE_DURATION) + fadeToWhite, 255),
            Math.min((int) (topClr.green * brightness / FADE_DURATION) + fadeToWhite, 255),
            Math.min((int) (topClr.blue * brightness / FADE_DURATION) + fadeToWhite, 255));
      }
      for (int i = (int) RobotConstants.get().led2023LedCount() / 2;
          i < RobotConstants.get().led2023LedCount();
          i++) {

        ledStrip.setRGB(
            i,
            Math.min((int) (bottomClr.red * brightness / FADE_DURATION) + fadeToWhite, 255),
            Math.min((int) (bottomClr.green * brightness / FADE_DURATION) + fadeToWhite, 255),
            Math.min((int) (bottomClr.blue * brightness / FADE_DURATION) + fadeToWhite, 255));
      }
    }
  }
}
