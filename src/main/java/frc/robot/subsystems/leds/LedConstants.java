package frc.robot.subsystems.leds;

import frc.robot.Constants;

public class LedConstants {
  public static final int LED_CHANNEL;
  public static final int LED_COUNT;

  public static final int LENGTH = 16;
  public static final int BASE1_START = 0;
  public static final int BASE1_END = LENGTH - 1;
  public static final int BAR_START = LENGTH;
  public static final int BAR_END = 2 * LENGTH - 1;
  public static final int BASE2_START = 2 * LENGTH;
  public static final int BASE2_END = 3 * LENGTH - 1;

  public static final int BASE1_FIRST_QUARTER_START = BASE1_START;
  public static final int BASE1_FIRST_QUARTER_END = (LENGTH / 4 - 1);
  public static final int BASE1_SECOND_QUARTER_START = (LENGTH / 4);
  public static final int BASE1_SECOND_QUARTER_END = (LENGTH * 2 / 4 - 1);
  public static final int BASE1_THIRD_QUARTER_START = (LENGTH * 2 / 4);
  public static final int BASE1_THIRD_QUARTER_END = (LENGTH * 3 / 4 - 1);
  public static final int BASE1_FOURTH_QUARTER_START = (LENGTH * 3 / 4);
  public static final int BASE1_FOURTH_QUARTER_END = (LENGTH - 1);

  public static final int BASE2_FOURTH_QUARTER_START = (BASE2_START);
  public static final int BASE2_FOURTH_QUARTER_END = (LENGTH / 4 - 1 + BASE2_START);
  public static final int BASE2_THIRD_QUARTER_START = (LENGTH / 4 + BASE2_START);
  public static final int BASE2_THIRD_QUARTER_END = (LENGTH * 2 / 4 - 1 + BASE2_START);
  public static final int BASE2_SECOND_QUARTER_START = (LENGTH * 2 / 4 + BASE2_START);
  public static final int BASE2_SECOND_QUARTER_END = (LENGTH * 3 / 4 - 1 + BASE2_START);
  public static final int BASE2_FIRST_QUARTER_START = (LENGTH * 3 / 4 + BASE2_START);
  public static final int BASE2_FIRST_QUARTER_END = (LENGTH - 1 + BASE2_START);

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
