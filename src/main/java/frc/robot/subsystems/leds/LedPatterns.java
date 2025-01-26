package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

public enum LedPatterns {
  BLACK(LEDPattern.solid(Color.kBlack)),
  RED(LEDPattern.solid(Color.kRed)),
  RAINBOW(LEDPattern.rainbow(255, 128));

  private static final double BREATH_TIME = 1.0;
  private static final double STROBE_TIME = 1.0;
  private static final Distance LED_SPACING = Meters.of(1 / 60.0);

  private final LEDPattern colorPattern;

  public LEDPattern colorPatternOnly() {
    return colorPattern;
  }

  public LEDPattern blink() {
    return colorPattern.blink(Seconds.of(STROBE_TIME));
  }

  public LEDPattern breathe() {
    return colorPattern.breathe(Seconds.of(BREATH_TIME));
  }

  public LEDPattern scroll() {
    return colorPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LED_SPACING);
  }

  private LedPatterns(LEDPattern colorPattern) {
    this.colorPattern = colorPattern;
  }

  // Solid
  public static final LEDPattern SOLID_ORANGE = LEDPattern.solid(Color.kOrange);
  public static final LEDPattern SOLID_YELLOW = LEDPattern.solid(Color.kYellow);
  public static final LEDPattern SOLID_GREEN = LEDPattern.solid(Color.kGreen);
  public static final LEDPattern SOLID_CYAN = LEDPattern.solid(Color.kCyan);
  public static final LEDPattern SOLID_BLUE = LEDPattern.solid(Color.kBlue);
  public static final LEDPattern SOLID_PURPLE = LEDPattern.solid(Color.kPurple);
  public static final LEDPattern SOLID_PINK = LEDPattern.solid(Color.kPink);
  public static final LEDPattern SOLID_MAGENTA = LEDPattern.solid(Color.kMagenta);
  public static final LEDPattern SOLID_BROWN = LEDPattern.solid(Color.kBrown);
  public static final LEDPattern SOLID_GRAY = LEDPattern.solid(Color.kGray);
  public static final LEDPattern SOLID_WHITE = LEDPattern.solid(Color.kWhite);
  public static final LEDPattern SOLID_GOLD = LEDPattern.solid(Color.kGold);
  public static final LEDPattern SOLID_SILVER = LEDPattern.solid(Color.kSilver);

  // Rainbow
  // public static final LEDPattern SCROLLING_RAINBOW =
  //     RAINBOW.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LED_SPACING);

  // Center of Mass
  public static final Color COM_BLUE = new Color(32, 42, 68);
  public static final Color COM_GOLD = new Color(197, 178, 88);
  public static final Color COM_WHITE = new Color(255, 255, 255);

  public static final LEDPattern SOLID_COM_BLUE = LEDPattern.solid(COM_BLUE);
  public static final LEDPattern SOLID_COM_GOLD = LEDPattern.solid(COM_GOLD);
  public static final LEDPattern SOLID_COM_WHITE = LEDPattern.solid(COM_WHITE);

  public static final LEDPattern GRADIENT_COM =
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, COM_BLUE, COM_GOLD, COM_WHITE);

  public static final LEDPattern STRIPE_COM =
      LEDPattern.steps(Map.of(0, COM_BLUE, 0.33, COM_GOLD, 0.66, COM_WHITE));

  // FRC
  public static final LEDPattern SOLID_FRC_BLUE = LEDPattern.solid(Color.kFirstBlue);
  public static final LEDPattern SOLID_FRC_RED = LEDPattern.solid(Color.kFirstRed);
  public static final LEDPattern SOLID_FRC_WHITE = SOLID_WHITE;

  public static final LEDPattern GRADIENT_FRC =
      LEDPattern.gradient(
          LEDPattern.GradientType.kContinuous, Color.kFirstBlue, Color.kWhite, Color.kFirstRed);

  public static final LEDPattern STRIPE_FRC =
      LEDPattern.steps(Map.of(0, Color.kFirstBlue, 0.33, Color.kWhite, 0.66, Color.kFirstRed));
}
