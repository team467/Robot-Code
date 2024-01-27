package frc.robot.subsystems.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.utils.VirtualSubsystem;
import frc.robot.subsystems.robotstate.RobotState;
import frc.robot.subsystems.robotstate.RobotStateIO;
import frc.robot.subsystems.robotstate.RobotStateIOInputsAutoLogged;

import java.util.List;

public class Leds extends VirtualSubsystem {
  private static Leds instance;
  
  private RobotStateIOInputsAutoLogged state;


  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds(RobotState.getInstance());
    }
    return instance;
  }

  // Robot state tracking
  public int loopCycleCount = 0;
  public boolean autoFinished = false;
  public double autoFinishedTime = 0.0;
  

  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;

  // Constants
  private static final int minLoopCycleCount = 10;
  private static final int length = 10;
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

  private Leds(RobotState robotState) {
    state = robotState.state();
    leds = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(length);
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

  @Override
  public void periodic() {

    

    // Update auto state
    if (DriverStation.isDisabled()) {
      autoFinished = false;
    } else {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getFPGATimestamp();
    }



    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

    // Select LED mode
    // solid(Section.FULL, Color.kBlack); // Default to off

    if (state.estopped) {
      solid(Section.FULL, Color.kRed);
    } else if (DriverStation.isDisabled()) {
      if (lastEnabledAuto && Timer.getFPGATimestamp() - lastEnabledTime < autoFadeMaxTime) {
        // Auto fade
        solid(1.0 - ((Timer.getFPGATimestamp() - lastEnabledTime) / autoFadeTime), Color.kGreen);
      } else if (state.lowBatteryAlert) {
        // Low battery
        solid(Section.FULL, Color.kOrangeRed);
      } else {
        // Default pattern
        switch (state.alliance) {
          case Red:
            wave(
                Section.FULL,
                Color.kRed,
                Color.kBlack,
                waveAllianceCycleLength,
                waveAllianceDuration);
            break;
          case Blue:
            // solid(0.5, Color.kLightGreen, Color.kBlack);
            wave(
                Section.FULL,
                Color.kBlue,
                Color.kBlack,
                waveAllianceCycleLength,
                waveAllianceDuration);
            break;
          default:
            wave(Section.FULL, Color.kGold, Color.kDarkBlue, waveSlowCycleLength, waveSlowDuration);
            break;
        }
      }
    } else if (DriverStation.isAutonomous()) {
      wave(Section.FULL, Color.kGold, Color.kDarkBlue, waveFastCycleLength, waveFastDuration);
      if (autoFinished) {
        double fullTime = (double) length / waveFastCycleLength * waveFastDuration;
        solid((Timer.getFPGATimestamp() - autoFinishedTime) / fullTime, Color.kGreen);
      }
    }

    leds.setData(buffer);
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
   * Applies a solid color to a given section of an LED strip. The section is filled with the
   * specified color.
   *
   * @param percent The percentage of the section to apply the solid color to. Value should be
   *     between 0.0 and 1.0.
   * @param color The color to be applied.
   */
  ;

  private void solid(double percent, Color color1, Color color2) {
    int color1Pixels = (int) Math.round(MathUtil.clamp(length * percent, 0, length));
    for (int i = 0; i < color1Pixels; i++) {
      buffer.setLED(i, color1);
    }
    for (int i = color1Pixels; i < length; i++) {
      buffer.setLED(i, color2);
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

  private void shooting() {
    wave(Section.FULL, Color.kMagenta, Color.kBlack, waveAllianceCycleLength, waveAllianceDuration);
    // leds glow in the direction it's shooting
  }

  private void intaking() {
    wave(Section.FULL, Color.kPurple, Color.kBlack, waveAllianceCycleLength, waveAllianceDuration);
    // leds glow in the direction it's intaking
  }

  private void hanging() {
    solid(Section.FULL, Color.kDarkGreen);
  }

  private void containing() {
    solid(Section.FULL, Color.kAquamarine);
  }

  private void leftnotedetection() {
    solid(0.5, Color.kYellow, Color.kBlack);
    // leds glow on left side
  }

  private void rightnotedetection() {
    solid(0.5, Color.kLightGreen, Color.kBlack);
    // leds glows on right side
  }
}
