package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.leds.LedConstants.*;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

public enum LedPatterns {
  BLACK(LEDPattern.solid(Color.kBlack)),
  RED(LEDPattern.solid(Color.kRed)),
  SOLID_ORANGE(LEDPattern.solid(Color.kOrange)),
  SOLID_YELLOW(LEDPattern.solid(Color.kYellow)),
  SOLID_GREEN(LEDPattern.solid(Color.kGreen)),
  SOLID_CYAN(LEDPattern.solid(Color.kCyan)),
  SOLID_BLUE(LEDPattern.solid(Color.kBlue)),
  SOLID_PURPLE(LEDPattern.solid(Color.kPurple)),
  SOLID_PINK(LEDPattern.solid(Color.kPink)),
  SOLID_MAGENTA(LEDPattern.solid(Color.kMagenta)),
  SOLID_BROWN(LEDPattern.solid(Color.kBrown)),
  SOLID_GRAY(LEDPattern.solid(Color.kGray)),
  SOLID_WHITE(LEDPattern.solid(Color.kWhite)),
  SOLID_GOLD(LEDPattern.solid(Color.kGold)),
  SOLID_SILVER(LEDPattern.solid(Color.kSilver)),

  RAINBOW(LEDPattern.rainbow(255, 128)),

  SOLID_COM_BLUE(LEDPattern.solid(new Color(32, 42, 68))),
  SOLID_COM_GOLD(LEDPattern.solid(new Color(197, 178, 88))),
  SOLID_COM_WHITE(LEDPattern.solid(new Color(255, 255, 255))),
  GRADIENT_COM(
      LEDPattern.gradient(
          LEDPattern.GradientType.kContinuous,
          new Color(32, 42, 68),
          new Color(197, 178, 88),
          new Color(255, 255, 255))),
  STRIPE_COM(
      LEDPattern.steps(
          Map.of(
              0,
              new Color(32, 42, 68),
              0.33,
              new Color(197, 178, 88),
              0.66,
              new Color(255, 255, 255)))),

  SOLID_FRC_BLUE(LEDPattern.solid(Color.kFirstBlue)),
  SOLID_FRC_RED(LEDPattern.solid(Color.kFirstRed)),
  SOLID_FRC_WHITE(LEDPattern.solid(Color.kWhite)),
  GRADIENT_FRC(
      LEDPattern.gradient(
          LEDPattern.GradientType.kContinuous, Color.kFirstBlue, Color.kWhite, Color.kFirstRed)),
  STRIPE_FRC(
      LEDPattern.steps(Map.of(0, Color.kFirstBlue, 0.33, Color.kWhite, 0.66, Color.kFirstRed)));

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
