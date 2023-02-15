package frc.lib.tuning;

import frc.robot.RobotConstants;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

/**
 * Class for a tunable string. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableString {
  private static final String tableKey = "TunableNumbers";

  private final String key;
  private boolean hasDefault = false;
  private String defaultValue;
  private LoggedDashboardString dashboardNumber;
  private final Map<Integer, String> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedTunableString(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public LoggedTunableString(String dashboardKey, String defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(String defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (RobotConstants.tuningMode) {
        dashboardNumber = new LoggedDashboardString(key, defaultValue);
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public String get() {
    if (!hasDefault) {
      return "";
    } else {
      return RobotConstants.tuningMode ? dashboardNumber.get() : defaultValue;
    }
  }

  /**
   * Checks whether the string has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the string has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    String currentValue = get();
    String lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || !Objects.equals(currentValue, lastValue)) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }
}
