package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

public enum LedPatterns {

  // Solid colors
  BLACK(LEDPattern.solid(Color.kBlack)),
  RED(LEDPattern.solid(Color.kRed)),
  ORANGE(LEDPattern.solid(Color.kOrange)),
  YELLOW(LEDPattern.solid(Color.kYellow)),
  GREEN(LEDPattern.solid(Color.kGreen)),
  CYAN(LEDPattern.solid(Color.kCyan)),
  BLUE(LEDPattern.solid(Color.kBlue)),
  PURPLE(LEDPattern.solid(Color.kPurple)),
  PINK(LEDPattern.solid(Color.kPink)),
  MAGENTA(LEDPattern.solid(Color.kMagenta)),
  BROWN(LEDPattern.solid(Color.kBrown)),
  GRAY(LEDPattern.solid(Color.kGray)),
  WHITE(LEDPattern.solid(Color.kWhite)),
  GOLD(LEDPattern.solid(Color.kGold)),
  SILVER(LEDPattern.solid(Color.kSilver)),

  // FRC
  FRC_BLUE(LEDPattern.solid(Color.kFirstBlue)),
  SOLID_FRC_RED(LEDPattern.solid(Color.kFirstRed)),

  // Center of Mass
  CENTER_OF_MASS_BLUE(LEDPattern.solid(new Color(32, 42, 68))),
  CENTER_OF_MASS_GOLD(LEDPattern.solid(new Color(197, 178, 88))),

  // Multi-Color Patterns
  RAINBOW(LEDPattern.rainbow(255, 128)),

  GRADIENT_COM(
      LEDPattern.gradient(
          LEDPattern.GradientType.kContinuous,
          new Color(32, 42, 68),
          new Color(197, 178, 88),
          Color.kWhite)),

  STRIPE_COM(
      LEDPattern.steps(
          Map.of(0, new Color(32, 42, 68), 0.33, new Color(197, 178, 88), 0.66, Color.kWhite))),

  GRADIENT_FRC(
      LEDPattern.gradient(
          LEDPattern.GradientType.kContinuous, Color.kFirstBlue, Color.kWhite, Color.kFirstRed)),
  STRIPE_FRC(
      LEDPattern.steps(Map.of(0, Color.kFirstBlue, 0.33, Color.kWhite, 0.66, Color.kFirstRed)));

  private static final double BREATH_TIME = 10.0;
  private static final double STROBE_TIME = 10.0;
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
}
