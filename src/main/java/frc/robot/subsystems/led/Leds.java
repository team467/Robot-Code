package frc.robot.subsystems.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;

public class Leds extends SubsystemBase {

  private RobotState state = RobotState.getInstance();

  public enum LedMode {
    ESTOPPED,
    AUTO_FINISHED,
    AUTONOMOUS,
    HANGING,
    IN_RANGE,
    CAN_SHOOT,
    SHOOTING,
    CONTAINING,
    INTAKING,
    LEFT_NOTE_DETECTION,
    RIGHT_NOTE_DETECTION,
    STRAIGHT_NOTE_DETECTION,
    BLUE_ALLIANCE,
    RED_ALLIANCE,
    LOW_BATTERY_ALERT,
    DISABLED,
    OFF,
  }

  @AutoLogOutput(key = "LEDs/Mode")
  LedMode mode = LedMode.OFF;

  // Robot state tracking
  public int loopCycleCount = 0;
  public double autoFinishedTime = 0.0;

  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;

  // Constants
  private static final int minLoopCycleCount = 10;
  private static final int length = 40;
  private static final double strobeFastDuration = 0.1;
  private static final double strobeSlowDuration = 0.2;
  private static final double breathDuration = 1.0;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveSlowCycleLength = 25.0;
  private static final double waveSlowDuration = 3.0;
  private static final double waveAllianceCycleLength = 15.0;
  private static final double waveAllianceDuration = 2.0;
  private static final double autoFadeTime = 2.5; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal
  private static final double noteAngle = 5.0;

  /** Creates a Network table for testing led modes and colors */
  private NetworkTable ledTable;
  /** Sets the mode for led in network table and allows to test led modes */
  private NetworkTableEntry ledModeEntry;
  /** Allows testing in leds by enabling testing mode */
  private NetworkTableEntry ledTestingEntry;

  public Leds() {
    ledTable = NetworkTableInstance.getDefault().getTable("Leds");
    ledModeEntry = ledTable.getEntry("Mode");
    ledModeEntry.setString("OFF");
    ledTestingEntry = ledTable.getEntry("Testing");
    ledTestingEntry.setBoolean(false);

    buffer = new AddressableLEDBuffer(length);

    leds = new AddressableLED(0);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();

    loadingNotifier =
        new Notifier(
            () -> {
              breath(
                  Section.FULL,
                  Color.kWhite,
                  Color.kBlack,
                  0.25,
                  (double) System.currentTimeMillis() / 1000);
              leds.setData(buffer);
            });
    loadingNotifier.startPeriodic(0.02);
  }

  private void updateState() {

    // Update auto state
    lastEnabledAuto = DriverStation.isAutonomous();
    lastEnabledTime = Timer.getFPGATimestamp();

    if (DriverStation.isEStopped()) {
      mode = LedMode.ESTOPPED;
    } else if (state.lowBatteryAlert) {
      mode = LedMode.LOW_BATTERY_ALERT;
      // low battery mode at top for testing purposes
    } else if (DriverStation.isDisabled()) {
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
          mode = LedMode.BLUE_ALLIANCE;
        } else {
          mode = LedMode.RED_ALLIANCE;
        }

      } else {
        mode = LedMode.DISABLED;
      }
    } else if (false) { // TODO: need state variable for auto finished
      mode = LedMode.AUTO_FINISHED;

    } else if (DriverStation.isAutonomous()) { // TODO: need state variable for autonomous
      mode = LedMode.AUTONOMOUS;

    } else if (state.hanging) {
      mode = LedMode.HANGING;

    } else if (state.shooting) {
      mode = LedMode.SHOOTING;

    } else if (state.canShoot && state.hasNote) {
      mode = LedMode.CAN_SHOOT;

    } else if (state.inRange && state.hasNote) {
      mode = LedMode.IN_RANGE;

    } else if (state.hasNote) {
      mode = LedMode.CONTAINING;

    } else if (state.intaking) {
      mode = LedMode.INTAKING;

    } else if (state.seeNote) {
      if (state.noteAngle <= -noteAngle) { // TODO: need to change to constant once angle known
        mode = LedMode.LEFT_NOTE_DETECTION;

      } else if (state.noteAngle >= noteAngle) {
        mode = LedMode.RIGHT_NOTE_DETECTION;

      } else {
        mode = LedMode.STRAIGHT_NOTE_DETECTION;
      }
    } else {
      mode = LedMode.OFF;
    }
  }

  private void updateLeds() {
    switch (mode) {
      case ESTOPPED:
        solid(Section.FULL, Color.kRed);
        // strobe(Section.FULL, Color.kRed, strobeFastDuration);
        break;

      case AUTO_FINISHED:
        double fullTime = (double) length / waveFastCycleLength * waveFastDuration;
        solid((Timer.getFPGATimestamp() - autoFinishedTime) / fullTime, Color.kGreen);
        break;

      case AUTONOMOUS:
        wave(Section.FULL, Color.kGold, Color.kDarkBlue, waveFastCycleLength, waveFastDuration);
        break;

      case HANGING:
        solid(Section.FULL, new Color("#006400")); // Dark Green is 0x006400
        break;

      case IN_RANGE:
        wave(
            Section.FULL,
            Color.kGreen,
            Color.kBlack,
            waveAllianceCycleLength,
            waveAllianceDuration);
        break;

      case CAN_SHOOT:
        solid(Section.FULL, Color.kGreen);
        // has the same color as shooting except it's solid
        break;

      case SHOOTING:
        // leds glow in the direction it's shooting
        wave(
            Section.FULL,
            Color.kGreen,
            Color.kBlack,
            waveAllianceCycleLength,
            waveAllianceDuration);
        break;

      case CONTAINING:
        solid(Section.FULL, Color.kGreen);
        break;

      case INTAKING:
        // leds glow in the direction it's intaking
        wave(
            Section.FULL,
            Color.kPurple,
            Color.kBlack,
            waveAllianceCycleLength,
            waveAllianceDuration);
        break;

      case LEFT_NOTE_DETECTION:
        // leds glow on left side
        solidOnSide(false, Color.kYellow);
        break;

      case RIGHT_NOTE_DETECTION:
        // leds glows on right side
        solidOnSide(true, Color.kYellow);
        break;

      case STRAIGHT_NOTE_DETECTION:
        // leds glow in the middle
        solidMiddle(0.5, Color.kYellowGreen);
        break;

      case BLUE_ALLIANCE:
        wave(
            Section.FULL, Color.kBlue, Color.kBlack, waveAllianceCycleLength, waveAllianceDuration);
        break;

      case RED_ALLIANCE:
        wave(Section.FULL, Color.kRed, Color.kBlack, waveAllianceCycleLength, waveAllianceDuration);
        break;

      case LOW_BATTERY_ALERT:
        strobe(Section.FULL, Color.kRed, 1);
        break;

      case DISABLED:
        if (lastEnabledAuto && Timer.getFPGATimestamp() - lastEnabledTime < autoFadeMaxTime) {
          // Auto fade
          solid(1.0 - ((Timer.getFPGATimestamp() - lastEnabledTime) / autoFadeTime), Color.kGreen);
        } else {
          wave(Section.FULL, Color.kGold, Color.kDarkBlue, waveSlowCycleLength, waveSlowDuration);
        }
        break;

      case OFF:
        wave(Section.FULL, Color.kGold, Color.kDarkBlue, waveSlowCycleLength, waveSlowDuration);
        break;

      default:
        wave(Section.FULL, Color.kGold, Color.kDarkBlue, waveSlowCycleLength, waveSlowDuration);
        break;
    }

    leds.setData(buffer);
  }

  @Override
  public void periodic() {

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }
    // Stop loading notifier if running
    loadingNotifier.stop();

    if (ledTestingEntry.getBoolean(false)) {
      mode = LedMode.valueOf(ledModeEntry.getString("OFF"));
    } else {
      updateState();
      ledModeEntry.setString(mode.toString());
    }
    updateLeds();
  }

  /**
   * Applies a solid color to a given section of an LED strip. The section is filled with the
   * specified color.
   *
   * @param section The section of the LED strip to apply the solid color to.
   * @param color The color to be applied.
   */
  private void solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        buffer.setLED(i, color);
      }
    }
  }

  /**
   * Applies a solid color to a given section of an LED strip. The section is filled with the
   * specified color.
   *
   * @param percent The percentage of the section to apply the solid color to. Value should be
   *     between 0.0 and 1.0.
   * @param color The color to be applied.
   */
  private void solid(double percent, Color color) {
    solid(percent, color, Color.kBlack);
  }

  /**
   * Applies a solid color to a side of the two LED strips.
   *
   * @param onRight should light up right side
   * @param color The color to be applied.
   */
  private void solidOnSide(boolean onRight, Color color) {
    if (onRight) {
      for (int i = 0; i < length / 2; i++) {
        buffer.setLED(i, Color.kBlack);
      }
      for (int i = length / 2; i < length; i++) {
        buffer.setLED(i, color);
      }
    } else { // On the left
      for (int i = 0; i < length / 2; i++) {
        buffer.setLED(i, color);
      }
      for (int i = length / 2; i < length; i++) {
        buffer.setLED(i, Color.kBlack);
      }
    }
  }

  /**
   * Applies a solid color to a given section of an LED strip. The section is filled with the
   * specified color.
   *
   * @param percent The percentage of the section to apply the solid color to. Value should be
   *     between 0.0 and 1.0.
   * @param color1 The color to be applied.
   */
  private void solid(double percent, Color color1, Color color2) {
    int color1Pixels = (int) Math.round(MathUtil.clamp(length * percent, 0, length));
    for (int i = 0; i < color1Pixels; i++) {
      buffer.setLED(i, color1);
    }
    for (int i = color1Pixels; i < length; i++) {
      buffer.setLED(i, color2);
    }
  }

  private void solidMiddle(double percent, Color color2) {
    double middlePercent = 1 - percent;
    int color1Pixels = (int) Math.round(MathUtil.clamp(length * (middlePercent / 2), 0, length));
    for (int i = 0; i < color1Pixels; i++) {
      buffer.setLED(i, Color.kBlack);
    }
    // middle is only part lighted up
    for (int i = color1Pixels; i < (length - color1Pixels); i++) {
      buffer.setLED(i, color2);
    }
    for (int i = (length - color1Pixels); i < length; i++) {
      buffer.setLED(i, Color.kBlack);
    }
  }

  /**
   * Changes the color of a section to create a strobe effect. The section alternates between the
   * given color and black based on the specified duration.
   *
   * @param section The section to apply the strobe effect to.
   * @param color The color to be displayed during the "on" state.
   * @param duration The duration of each on-off cycle, in seconds.
   */
  private void strobe(Section section, Color color, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(section, on ? color : Color.kBlack);
  }

  /**
   * Changes the color of a section to create a breathing effect. The color gradually transitions
   * from c1 to c2 and back to c1 in a sine wave pattern.
   *
   * @param section The section to apply the breathing effect to.
   * @param c1 The initial color of the section.
   * @param c2 The target color of the section.
   * @param duration The total duration of the breathing effect, in seconds.
   */
  private void breath(Section section, Color c1, Color c2, double duration) {
    breath(section, c1, c2, duration, Timer.getFPGATimestamp());
  }

  /**
   * Changes the color of a section to create a breathing effect. The color gradually transitions
   * from c1 to c2 and back to c1 in a sine wave pattern.
   *
   * @param section The section to apply the breathing effect to.
   * @param c1 The initial color of the section.
   * @param c2 The target color of the section.
   * @param duration The total duration of the breathing effect, in seconds.
   * @param timestamp The current timestamp in seconds.
   */
  private void breath(Section section, Color c1, Color c2, double duration, double timestamp) {
    double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(section, new Color(red, green, blue));
  }

  /**
   * Applies a rainbow effect to a given section of an LED strip.
   *
   * @param section the section of the LED strip to apply the rainbow effect to
   * @param cycleLength the length of a complete rainbow cycle in degrees
   * @param duration the duration of the rainbow effect in seconds
   */
  private void rainbow(Section section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (i >= section.start()) {
        buffer.setHSV(i, (int) x, 255, 255);
      }
    }
  }

  /**
   * Wave method applies wave effect to a given section of an LED strip. The wave effect creates a
   * smooth transition of colors between two given colors over a specified cycle length and
   * duration.
   *
   * @param section the section of the LED strip to apply the wave effect to
   * @param c1 the starting color of the wave effect
   * @param c2 the ending color of the wave effect
   * @param cycleLength the length of a complete wave cycle in radians
   * @param duration the duration of the wave effect in seconds
   */
  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      if (i >= section.start()) {
        double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        buffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  /**
   * Applies a stripe effect to a given section of an LED strip. The stripe effect creates a pattern
   * of stripes with different colors.
   *
   * @param section the section of the LED strip to apply the stripe effect to
   * @param colors the list of colors for the stripes
   * @param length the number of sequential LEDs to be the same color
   * @param duration the duration of the stripe effect in seconds
   */
  private void stripes(Section section, List<Color> colors, int length, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
    for (int i = section.start(); i < section.end(); i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  private enum Section {
    FULL,
    TOP_HALF,
    BOTTOM_HALF;

    private int start() {
      switch (this) {
        case FULL:
          return 0;
        case TOP_HALF:
          return 0;
        case BOTTOM_HALF:
          return length / 2;
        default:
          return 0;
      }
    }

    private int end() {
      switch (this) {
        case FULL:
          return length;
        case TOP_HALF:
          return length / 2;
        case BOTTOM_HALF:
          return length;
        default:
          return length;
      }
    }
  }
}
