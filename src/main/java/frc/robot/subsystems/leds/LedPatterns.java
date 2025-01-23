package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

public enum LedPatterns {
  SOLID_BLUE(LEDPattern.solid(Color.kBlue)),
  SOLID_RED(LEDPattern.solid(Color.kRed)),
  SOLID_ORANGE_RED(LEDPattern.solid(Color.kOrangeRed)),
  SOLID_WHITE(LEDPattern.solid(Color.kWhite)),
  SCROLLING_RAINBOW(
      LEDPattern.rainbow(255, 128)
          .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LedConstants.LED_SPACING)),
  CONTINUOUS_GRADIENT_FROM_RED_TO_BLUE(
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kBlue)),
  DISCONTINUOUS_GRADIENT_FROM_HOTPINK_TO_LIME(
      LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kHotPink, Color.kLime)),
  STEPS(LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlue))),
  BREATH(LEDPattern.solid(Color.kHotPink));

  public final LEDPattern pattern;

  private LedPatterns(LEDPattern pattern) {
    this.pattern = pattern;
  }
}
