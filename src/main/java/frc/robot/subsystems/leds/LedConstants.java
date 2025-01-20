package frc.robot.subsystems.leds;

import frc.robot.Constants;

public class LedConstants {
  public static final int LED_CHANNEL;
  public static final int LED_COUNT;

  public static final int MIN_LOOP_CYCLE_COUNT = 10;
  public static final int LENGTH = 40;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2025_COMP -> {
        LED_CHANNEL = 0;
        LED_COUNT = 100;
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
