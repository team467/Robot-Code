package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class CustomTriggers {
  private static final double JOYSTICK_THRESHOLD = 0.2;
  private static final Map<Trigger, Boolean> lastTriggerValues = new HashMap<>();
  private static final Map<Trigger, Boolean> toggledState = new HashMap<>();

  public Trigger toggleOnTrueCancelableWithJoystick(
      Trigger buttonInput, DoubleSupplier X, DoubleSupplier Y) {

    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(),
        () -> {
          boolean toggledState = this.toggledState.getOrDefault(buttonInput, false);
          boolean joystickEngaged =
              Math.hypot(Math.abs(X.getAsDouble()), Math.abs(Y.getAsDouble())) > JOYSTICK_THRESHOLD;

          boolean lastValue = lastTriggerValues.getOrDefault(buttonInput, false);
          boolean currentValue = buttonInput.getAsBoolean();

          if (joystickEngaged) {
            toggledState = false;
            this.toggledState.put(buttonInput, toggledState);
            lastTriggerValues.put(buttonInput, false);
            return false;
          }

          boolean stateChanged = currentValue && !lastValue;

          if (stateChanged) {
            toggledState = !toggledState;
          }

          lastTriggerValues.put(buttonInput, currentValue);
          this.toggledState.put(buttonInput, toggledState);
          return toggledState;
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
          boolean toggledState = this.toggledState.getOrDefault(buttonInput, false);
          boolean joystickEngaged =
              Math.hypot(Math.abs(X1.getAsDouble()), Math.abs(Y1.getAsDouble()))
                      > JOYSTICK_THRESHOLD
                  || Math.hypot(Math.abs(X2.getAsDouble()), Math.abs(Y2.getAsDouble()))
                      > JOYSTICK_THRESHOLD;

          boolean lastValue = lastTriggerValues.getOrDefault(buttonInput, false);
          boolean currentValue = buttonInput.getAsBoolean();

          if (joystickEngaged) {
            toggledState = false;
            this.toggledState.put(buttonInput, toggledState);
            lastTriggerValues.put(buttonInput, false);
            return false;
          }

          boolean stateChanged = currentValue && !lastValue;

          if (stateChanged) {
            toggledState = !toggledState;
          }

          lastTriggerValues.put(buttonInput, currentValue);
          this.toggledState.put(buttonInput, toggledState);
          return toggledState;
        });
  }
}
