package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class CustomTriggers {
  private static final double JOYSTICK_THRESHOLD = 0.2;
  private Map<Trigger, Boolean> lastTriggerValues = new HashMap<>();

  public Trigger toggleOnTrueCancelableWithJoystick(
      Trigger buttonInput, DoubleSupplier X, DoubleSupplier Y) {
    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(),
        () -> {
          boolean joystickEngaged =
              Math.abs(X.getAsDouble()) > JOYSTICK_THRESHOLD || Math.abs(Y.getAsDouble()) > JOYSTICK_THRESHOLD;
          boolean lastValue = lastTriggerValues.getOrDefault(buttonInput, false);

          if (!joystickEngaged) {
            boolean currentValue = buttonInput.getAsBoolean();
            lastTriggerValues.put(buttonInput, currentValue);
            return currentValue != lastValue;
          }
          lastTriggerValues.put(buttonInput, false);
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
          boolean lastValue = lastTriggerValues.getOrDefault(buttonInput, false);

          if (!joystickEngaged) {
            boolean currentValue = buttonInput.getAsBoolean();
            lastTriggerValues.put(buttonInput, currentValue);
            return currentValue != lastValue;
          }
          lastTriggerValues.put(buttonInput, false);
          return false;
        });
  }
}
