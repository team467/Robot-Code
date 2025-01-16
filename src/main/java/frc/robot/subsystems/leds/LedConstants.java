package frc.robot.subsystems.leds;

import frc.robot.Constants;

public class LedConstants {
  public static final int LED_CHANNEL;
  public static final int LED_COUNT;

  public static final int MIN_LOOP_CYCLE_COUNT = 10;
  public static final int LENGTH = 40;
  public static final double STROBE_FAST_DURATION = 0.1;
  public static final double STROBE_SLOW_DURATION = 0.2;
  public static final double BREATH_DURATION = 1.0;
  public static final double RAINBOW_CYCLE_LENGTH = 25.0;
  public static final double RAINBOW_DURATION = 0.25;
  public static final double WAVE_EXPONENT = 0.4;
  public static final double WAVE_FAST_CYCLE_LENGTH = 25.0;
  public static final double WAVE_FAST_DURATION = 0.25;
  public static final double WAVE_SLOW_CYCLE_LENGTH = 25.0;
  public static final double WAVE_SLOW_DURATION = 3.0;
  public static final double WAVE_ALLIANCE_CYCLE_LENGTH = 15.0;
  public static final double WAVE_ALLIANCE_DURATION = 2.0;
  public static final double AUTO_FADE_TIME = 2.5; // 3s nominal
  public static final double AUTO_FADE_MAX_TIME = 5.0; // Return to normal

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2025_COMP -> {
        LED_CHANNEL = 0;
        LED_COUNT = 100;
      }
      case ROBOT_2023 -> {
        LED_CHANNEL = 0;
        LED_COUNT = 10;
      }
      case ROBOT_BRIEFCASE -> {
        LED_CHANNEL = 0;
        LED_COUNT = 10;
      }
      default -> {
        LED_CHANNEL = 0;
        LED_COUNT = 0;
      }
    }
  }
}
