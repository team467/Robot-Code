package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class CustomTriggers {
  private static final double JOYSTICK_THRESHOLD = 0.2;
  private static final Map<Trigger, Boolean> mLastTriggerValues = new HashMap<>();
  private static final Map<Trigger, Boolean> mToggledState = new HashMap<>();

  public static Trigger toggleOnTrueCancelableWithJoystick(
      Trigger buttonInput, DoubleSupplier X, DoubleSupplier Y) {

    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(),
        () -> {
          boolean toggledState = mToggledState.getOrDefault(buttonInput, false);
          boolean joystickEngaged =
              Math.hypot(Math.abs(X.getAsDouble()), Math.abs(Y.getAsDouble())) > JOYSTICK_THRESHOLD;

          boolean lastValue = mLastTriggerValues.getOrDefault(buttonInput, false);
          boolean currentValue = buttonInput.getAsBoolean();

          if (joystickEngaged) {
            toggledState = false;
            mToggledState.put(buttonInput, toggledState);
            mLastTriggerValues.put(buttonInput, false);
            return false;
          }

          boolean stateChanged = currentValue && !lastValue;

          if (stateChanged) {
            toggledState = !toggledState;
          }

          mLastTriggerValues.put(buttonInput, currentValue);
          mToggledState.put(buttonInput, toggledState);
          return toggledState;
        });
  }

  public static Trigger toggleOnTrueCancelableWithJoysticks(
      Trigger buttonInput,
      DoubleSupplier X1,
      DoubleSupplier Y1,
      DoubleSupplier X2,
      DoubleSupplier Y2) {

    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(),
        () -> {
          boolean toggledState = mToggledState.getOrDefault(buttonInput, false);
          boolean joystickEngaged =
              Math.hypot(Math.abs(X1.getAsDouble()), Math.abs(Y1.getAsDouble()))
                      > JOYSTICK_THRESHOLD
                  || Math.hypot(Math.abs(X2.getAsDouble()), Math.abs(Y2.getAsDouble()))
                      > JOYSTICK_THRESHOLD;

          boolean lastValue = mLastTriggerValues.getOrDefault(buttonInput, false);
          boolean currentValue = buttonInput.getAsBoolean();

          if (joystickEngaged) {
            toggledState = false;
            mToggledState.put(buttonInput, toggledState);
            mLastTriggerValues.put(buttonInput, false);
            return false;
          }

          boolean stateChanged = currentValue && !lastValue;

          if (stateChanged) {
            toggledState = !toggledState;
          }

          mLastTriggerValues.put(buttonInput, currentValue);
          mToggledState.put(buttonInput, toggledState);
          return toggledState;
        });
  }
  public Trigger manualModeInput(Trigger inputButton, Trigger correctMode){
    return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(),
        () ->
        {
          return inputButton.getAsBoolean() && correctMode.getAsBoolean();
        }
        );
  }
}
