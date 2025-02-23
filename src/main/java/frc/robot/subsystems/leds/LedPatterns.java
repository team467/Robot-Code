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
  FRC_RED(LEDPattern.solid(Color.kFirstRed)),

  // Center of Mass
  CENTER_OF_MASS_BLUE(LEDPattern.solid(Color.kDarkBlue)),
  CENTER_OF_MASS_GOLD(LEDPattern.solid(Color.kDarkGoldenrod)),

  // Multi-Color Patterns
  RAINBOW(LEDPattern.rainbow(255, 128)),

  // Game Specific
  STRIPE_ALGAE(
      LEDPattern.steps(
          Map.of(
              0, Color.kGreen, 0.25, Color.kLimeGreen, 0.5, Color.kGreen, 0.75, Color.kLimeGreen))),

  STRIPE_CORAL(
      LEDPattern.steps(
          Map.of(
              0,
              Color.kPurple,
              0.25,
              Color.kMediumPurple,
              0.5,
              Color.kPurple,
              0.75,
              Color.kMediumPurple))),

  GRADIENT_COM(
      LEDPattern.gradient(
          LEDPattern.GradientType.kContinuous, Color.kDarkBlue, Color.kDarkGoldenrod)),

  STRIPE_COM(LEDPattern.steps(Map.of(0, Color.kDarkBlue, 0.50, Color.kDarkGoldenrod))),

  GRADIENT_FRC(
      LEDPattern.gradient(
          LEDPattern.GradientType.kContinuous, Color.kFirstBlue, Color.kWhite, Color.kFirstRed)),
  STRIPE_FRC(
      LEDPattern.steps(Map.of(0, Color.kFirstBlue, 0.33, Color.kWhite, 0.66, Color.kFirstRed)));

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

  public LEDPattern blink(double time) {
    return colorPattern.blink(Seconds.of(time > 0.0 ? time : STROBE_TIME));
  }

  public LEDPattern breathe() {
    return colorPattern.breathe(Seconds.of(BREATH_TIME));
  }

  public LEDPattern breathe(double time) {
    return colorPattern.breathe(Seconds.of(time > 0.0 ? time : BREATH_TIME));
  }

  public LEDPattern scroll() {
    return colorPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LED_SPACING);
  }

  public LEDPattern scroll(double metersPerSecond) {
    return colorPattern.scrollAtAbsoluteSpeed(
        MetersPerSecond.of(metersPerSecond > 0.0 ? metersPerSecond : 1), LED_SPACING);
  }

  public LEDPattern overlayOn(LEDPattern base) {
    return colorPattern.overlayOn(base);
  }

  public LEDPattern blend(LEDPattern other) {
    return colorPattern.blend(other);
  }

  private LedPatterns(LEDPattern colorPattern) {
    this.colorPattern = colorPattern;
  }
}
