package first.robot.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public final class PhoenixUtil {
  private PhoenixUtil() {}

  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) {
        break;
      }
    }
  }
}
