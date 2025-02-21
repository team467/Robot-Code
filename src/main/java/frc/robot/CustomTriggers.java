package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class CustomTriggers {
  private static final double JOYSTICK_THRESHOLD = 0.2;
  private final Map<Trigger, Boolean> lastButtonValues = new HashMap<>();

  public Trigger toggleOnTrueCancelableWithJoystick(
      Trigger buttonInput, DoubleSupplier X, DoubleSupplier Y) {
    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(),
        () -> {
          boolean joystickEngaged =
              Math.abs(X.getAsDouble()) > JOYSTICK_THRESHOLD
                  || Math.abs(Y.getAsDouble()) > JOYSTICK_THRESHOLD;
          boolean lastValue = lastButtonValues.getOrDefault(buttonInput, false);

          if (!joystickEngaged) {
            boolean currentValue = buttonInput.getAsBoolean();
            lastButtonValues.put(buttonInput, currentValue);
            return currentValue != lastValue;
          }
          lastButtonValues.put(buttonInput, false);
          return false;
        });
  }

  public Trigger toggleOnTrueCancelableWithJoysticks(
      Trigger buttonInput,
      DoubleSupplier X1,
      DoubleSupplier Y1,
      DoubleSupplier X2,
      DoubleSupplier Y2) {
    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(),
        () -> {
          boolean joystickEngaged =
              Math.abs(X1.getAsDouble()) > JOYSTICK_THRESHOLD
                  || Math.abs(Y1.getAsDouble()) > JOYSTICK_THRESHOLD
                  || Math.abs(X2.getAsDouble()) > JOYSTICK_THRESHOLD
                  || Math.abs(Y2.getAsDouble()) > JOYSTICK_THRESHOLD;
          boolean lastValue = lastButtonValues.getOrDefault(buttonInput, false);

          if (!joystickEngaged) {
            boolean currentValue = buttonInput.getAsBoolean();
            lastButtonValues.put(buttonInput, currentValue);
            return currentValue != lastValue;
          }
          lastButtonValues.put(buttonInput, false);
          return false;
        });
  }
}
