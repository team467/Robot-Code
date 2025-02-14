package frc.robot.subsystems.leds;

import frc.robot.Constants;

public class LedConstants {
  public static final int LED_CHANNEL;
  public static final int LED_COUNT;

  /* num LEDs - change the first two numbers based on actual led strip */
  public static final int FULL_LENGTH = 81;
  public static final int BAR_LENGTH = Math.min(27, FULL_LENGTH);
  public static final int BASE_LENGTH = (FULL_LENGTH - BAR_LENGTH) / 2;

  public static final int BASE1_START = 0;
  public static final int BASE1_END = BASE_LENGTH - 1;
  public static final int BAR_START = BASE1_END + 1;
  public static final int BAR_END = (BAR_START + BAR_LENGTH) - 1;
  public static final int BASE2_START = BAR_END + 1;
  public static final int BASE2_END = (BASE2_START + BASE_LENGTH) - 1;

  public static final int BASE1_FIRST_QUARTER_START = BASE1_START;
  public static final int BASE1_FIRST_QUARTER_END = (BASE_LENGTH / 4 - 1);
  public static final int BASE1_SECOND_QUARTER_START = (BASE_LENGTH / 4);
  public static final int BASE1_SECOND_QUARTER_END = (BASE_LENGTH * 2 / 4 - 1);
  public static final int BASE1_THIRD_QUARTER_START = (BASE_LENGTH * 2 / 4);
  public static final int BASE1_THIRD_QUARTER_END = (BASE_LENGTH * 3 / 4 - 1);
  public static final int BASE1_FOURTH_QUARTER_START = (BASE_LENGTH * 3 / 4);
  public static final int BASE1_FOURTH_QUARTER_END = (BASE_LENGTH - 1);

  public static final int BASE2_FOURTH_QUARTER_START = (BASE2_START);
  public static final int BASE2_FOURTH_QUARTER_END = (BASE_LENGTH / 4 - 1 + BASE2_START);
  public static final int BASE2_THIRD_QUARTER_START = (BASE_LENGTH / 4 + BASE2_START);
  public static final int BASE2_THIRD_QUARTER_END = (BASE_LENGTH * 2 / 4 - 1 + BASE2_START);
  public static final int BASE2_SECOND_QUARTER_START = (BASE_LENGTH * 2 / 4 + BASE2_START);
  public static final int BASE2_SECOND_QUARTER_END = (BASE_LENGTH * 3 / 4 - 1 + BASE2_START);
  public static final int BASE2_FIRST_QUARTER_START = (BASE_LENGTH * 3 / 4 + BASE2_START);
  public static final int BASE2_FIRST_QUARTER_END = (BASE_LENGTH - 1 + BASE2_START);

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
