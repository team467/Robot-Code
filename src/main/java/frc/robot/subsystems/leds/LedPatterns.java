package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LedPatterns {

  public static final Distance LED_SPACING = Meters.of(1 / 60.0);
  public static final LEDPattern BASE =
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kBlue)
          .scrollAtAbsoluteSpeed(Centimeters.per(Second).of(15), LED_SPACING);
  LEDPattern pattern = BASE.scrollAtRelativeSpeed(Percent.per(Second).of(25));
  LEDPattern absolute = BASE.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), LED_SPACING);
  public static final LEDPattern BREATH = LEDPattern.solid(Color.kWhiteSmoke);
  public static final LEDPattern ESTOPPED = LEDPattern.solid(Color.kRed);
  public static final LEDPattern AUTO_FINISHED = LEDPattern.solid(Color.kOrange);
  public static final LEDPattern AUTONOMOUS =
      LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LED_SPACING);
  public static final LEDPattern BLUE_ALLIANCE = LEDPattern.solid(Color.kBlue);
  public static final LEDPattern RED_ALLIANCE = LEDPattern.solid(Color.kRed);
  public static final LEDPattern LOW_BATTERY_ALERT = LEDPattern.solid(Color.kAquamarine);
  public static final LEDPattern DISABLED = LEDPattern.solid(Color.kLavender);
  public static final LEDPattern OFF = LEDPattern.solid(Color.kDarkViolet);
  public static final LEDPattern DEFAULT = LEDPattern.solid(Color.kDeepPink);
  public static final LEDPattern BASE2 =
      LEDPattern.gradient(
          LEDPattern.GradientType.kDiscontinuous, Color.kDarkTurquoise, Color.kDarkViolet);
  LEDPattern LedPattern = BASE2.breathe(Second.of(2));
}
