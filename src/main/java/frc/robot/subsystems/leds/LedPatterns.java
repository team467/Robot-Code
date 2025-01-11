package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LedPatterns {

  public static final LEDPattern BREATH = LEDPattern.solid(Color.kWhiteSmoke);

  public static final LEDPattern ESTOPPED = LEDPattern.solid(Color.kRed);
  public static final LEDPattern AUTO_FINISHED = LEDPattern.solid(Color.kOrange);
  public static final LEDPattern AUTONOMOUS = LEDPattern.solid(Color.kLightGoldenrodYellow);
  public static final LEDPattern BLUE_ALLIANCE = LEDPattern.solid(Color.kGreen);
  public static final LEDPattern RED_ALLIANCE = LEDPattern.solid(Color.kLightSkyBlue);
  public static final LEDPattern LOW_BATTERY_ALERT = LEDPattern.solid(Color.kAquamarine);
  public static final LEDPattern DISABLED = LEDPattern.solid(Color.kLavender);
  public static final LEDPattern OFF = LEDPattern.solid(Color.kDarkViolet);
  public static final LEDPattern DEFAULT = LEDPattern.solid(Color.kDeepPink);
}
