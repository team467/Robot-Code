package frc.robot.subsystems.leds;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Seconds;

import java.util.Map;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.hardware.led.LEDPattern.GradientType;
import org.wpilib.units.measure.Distance;
import org.wpilib.util.Color;

public enum LedPatterns {

  // Solid colors
  BLACK(LEDPattern.solid(Color.BLACK)),
  RED(LEDPattern.solid(Color.RED)),
  ORANGE(LEDPattern.solid(Color.ORANGE)),
  YELLOW(LEDPattern.solid(Color.YELLOW)),
  GREEN(LEDPattern.solid(Color.GREEN)),
  CYAN(LEDPattern.solid(Color.CYAN)),
  BLUE(LEDPattern.solid(Color.BLUE)),
  PURPLE(LEDPattern.solid(Color.PURPLE)),
  PINK(LEDPattern.solid(Color.PINK)),
  MAGENTA(LEDPattern.solid(Color.MAGENTA)),
  BROWN(LEDPattern.solid(Color.BROWN)),
  GRAY(LEDPattern.solid(Color.GRAY)),
  WHITE(LEDPattern.solid(Color.WHITE)),
  GOLD(LEDPattern.solid(Color.GOLD)),
  SILVER(LEDPattern.solid(Color.SILVER)),
  INDIAN_RED(LEDPattern.solid(Color.INDIAN_RED)),

  // FRC
  FRC_BLUE(LEDPattern.solid(Color.FIRST_BLUE)),
  FRC_RED(LEDPattern.solid(Color.FIRST_RED)),

  // Center of Mass
  CENTER_OF_MASS_BLUE(LEDPattern.solid(Color.DARK_BLUE)),
  CENTER_OF_MASS_GOLD(LEDPattern.solid(Color.DARK_GOLDENROD)),

  // Multi-Color Patterns
  RAINBOW(LEDPattern.rainbow(255, 128)),

  GRADIENT_COM(
      LEDPattern.gradient(
          GradientType.CONTINUOUS, Color.DARK_BLUE, Color.DARK_GOLDENROD)),

  STRIPE_COM(LEDPattern.steps(Map.of(0, Color.DARK_BLUE, 0.50, Color.DARK_GOLDENROD))),

  GRADIENT_FRC(
      LEDPattern.gradient(
          GradientType.CONTINUOUS, Color.FIRST_BLUE, Color.WHITE, Color.FIRST_RED)),

  STRIPE_FRC(
      LEDPattern.steps(Map.of(0, Color.FIRST_BLUE, 0.33, Color.WHITE, 0.66, Color.FIRST_RED))),

  STRIPE_YELOW_WHITE_BLUE(
      LEDPattern.steps(Map.of(0, Color.YELLOW, 0.33, Color.WHITE, 0.66, Color.BLUE))),
  STRIPE_YELLOW_WHITE(LEDPattern.steps(Map.of(0, Color.YELLOW, 0.50, Color.WHITE))),
  STRIPE_YELLOW_BLUE(LEDPattern.steps(Map.of(0, Color.YELLOW, 0.50, Color.BLUE))),
  STRIPE_BLUE_WHITE(LEDPattern.steps(Map.of(0, Color.BLUE, 0.50, Color.WHITE)));

  private static final double BREATH_TIME = 1.0;
  private static final double STROBE_TIME = 1.0;
  private static final Distance LED_SPACING = Meters.of(1 / 60.0);

  private final LEDPattern colorPattern;

  LedPatterns(LEDPattern colorPattern) {
    this.colorPattern = colorPattern;
  }

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

  public LEDPattern scroll(double metersPerSecond) {
    return colorPattern.scrollAtAbsoluteVelocity(
        MetersPerSecond.of(metersPerSecond > 0.0 ? metersPerSecond : 1), LED_SPACING);
  }

  public LEDPattern overlayOn(LEDPattern base) {
    return colorPattern.overlayOn(base);
  }

  public LEDPattern blend(LEDPattern other) {
    return colorPattern.blend(other);
  }
}
