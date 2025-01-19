package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.leds.LedConstants.*;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

public class LedPatterns {
  // Solid
  public static final LEDPattern SOLID_RED = LEDPattern.solid(Color.kRed);
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
  public static final LEDPattern SOLID_BLACK = LEDPattern.solid(Color.kBlack);
  public static final LEDPattern SOLID_GOLD = LEDPattern.solid(Color.kGold);
  public static final LEDPattern SOLID_SILVER = LEDPattern.solid(Color.kSilver);

  // Strobing
  public static final LEDPattern STROBE_RED = SOLID_RED.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_ORANGE = SOLID_ORANGE.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_YELLOW = SOLID_YELLOW.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_GREEN = SOLID_GREEN.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_CYAN = SOLID_CYAN.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_BLUE = SOLID_BLUE.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_PURPLE = SOLID_PURPLE.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_PINK = SOLID_PINK.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_MAGENTA = SOLID_MAGENTA.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_BROWN = SOLID_BROWN.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_GRAY = SOLID_GRAY.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_WHITE = SOLID_WHITE.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_BLACK = SOLID_BLACK.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_GOLD = SOLID_GOLD.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_SILVER = SOLID_SILVER.blink(Seconds.of(STROBE_TIME));

  // Breathing
  public static final LEDPattern BREATH_RED = SOLID_RED.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATH_ORANGE = SOLID_ORANGE.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATH_YELLOW = SOLID_YELLOW.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATH_GREEN = SOLID_GREEN.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATH_CYAN = SOLID_CYAN.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATH_BLUE = SOLID_BLUE.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATH_PURPLE = SOLID_PURPLE.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATH_PINK = SOLID_PINK.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATH_MAGENTA = SOLID_MAGENTA.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATH_BROWN = SOLID_BROWN.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATH_GRAY = SOLID_GRAY.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATH_WHITE = SOLID_WHITE.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATH_BLACK = SOLID_BLACK.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATH_GOLD = SOLID_GOLD.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATH_SILVER = SOLID_SILVER.breathe(Seconds.of(BREATH_TIME));

  // Rainbow
  public static final LEDPattern RAINBOW = LEDPattern.rainbow(255, 128);
  public static final LEDPattern SCROLLING_RAINBOW =
      RAINBOW.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LED_SPACING);

  // Center of Mass
  public static final Color COM_BLUE = new Color(32, 42, 68);
  public static final Color COM_GOLD = new Color(197, 178, 88);
  public static final Color COM_WHITE = new Color(255, 255, 255);

  public static final LEDPattern SOLID_COM_BLUE = LEDPattern.solid(COM_BLUE);
  public static final LEDPattern SOLID_COM_GOLD = LEDPattern.solid(COM_GOLD);
  public static final LEDPattern SOLID_COM_WHITE = LEDPattern.solid(COM_WHITE);

  public static final LEDPattern STROBE_COM_BLUE = SOLID_COM_BLUE.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_COM_GOLD = SOLID_COM_GOLD.blink(Seconds.of(STROBE_TIME));
  public static final LEDPattern STROBE_COM_WHITE = SOLID_COM_WHITE.blink(Seconds.of(STROBE_TIME));

  public static final LEDPattern BREATHE_COM_BLUE = SOLID_COM_BLUE.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATHE_COM_GOLD = SOLID_COM_GOLD.breathe(Seconds.of(BREATH_TIME));
  public static final LEDPattern BREATHE_COM_WHITE =
      SOLID_COM_WHITE.breathe(Seconds.of(BREATH_TIME));

  public static final LEDPattern GRADIENT_COM =
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, COM_BLUE, COM_GOLD, COM_WHITE);
  public static final LEDPattern SCROLLING_GRADIENT_COM =
      GRADIENT_COM.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LED_SPACING);

  public static final LEDPattern STRIPE_COM =
      LEDPattern.steps(Map.of(0, COM_BLUE, 0.33, COM_GOLD, 0.66, COM_WHITE));
  public static final LEDPattern SCROLLING_STRIPE_COM =
      STRIPE_COM.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LED_SPACING);
}
