package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class CustomTriggers {
  private static final double JOYSTICK_THRESHOLD = 0.2;
  private final Map<Trigger, Boolean> lastTriggerValues = new HashMap<>();

  public Trigger toggleOnTrueCancelableWithJoystick(
      Trigger buttonInput, DoubleSupplier X, DoubleSupplier Y) {

    AtomicBoolean toggledState = new AtomicBoolean(false);

    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(),
        () -> {
          boolean joystickEngaged =
              Math.hypot(Math.abs(X.getAsDouble()), Math.abs(Y.getAsDouble())) > JOYSTICK_THRESHOLD;

          boolean lastValue = lastTriggerValues.getOrDefault(buttonInput, false);
          boolean currentValue = buttonInput.getAsBoolean();

          Logger.recordOutput("CustomJoysticks/Joystick Engaged", joystickEngaged);
          Logger.recordOutput("CustomJoysticks/Last Value", lastValue);
          Logger.recordOutput("CustomJoysticks/Current Value", currentValue);

          if (joystickEngaged) {
            toggledState.set(false);
            lastTriggerValues.put(buttonInput, false);
            Logger.recordOutput("CustomJoysticks/Returns", false);
            return false;
          }

          boolean stateChanged = currentValue && !lastValue;
          Logger.recordOutput("CustomJoysticks/State Changed", stateChanged);

          if (stateChanged) {
            toggledState.set(!toggledState.get());
          }

          lastTriggerValues.put(buttonInput, currentValue);
          Logger.recordOutput("CustomJoysticks/Returns", toggledState.get());
          return toggledState.get();
        });
  }

  public Trigger toggleOnTrueCancelableWithJoysticks(
      Trigger buttonInput,
      DoubleSupplier X1,
      DoubleSupplier Y1,
      DoubleSupplier X2,
      DoubleSupplier Y2) {

    AtomicBoolean toggledState = new AtomicBoolean(false);

    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(),
        () -> {
          boolean joystickEngaged =
              Math.hypot(Math.abs(X1.getAsDouble()), Math.abs(Y1.getAsDouble()))
                      > JOYSTICK_THRESHOLD
                  || Math.hypot(Math.abs(X2.getAsDouble()), Math.abs(Y2.getAsDouble()))
                      > JOYSTICK_THRESHOLD;

          boolean lastValue = lastTriggerValues.getOrDefault(buttonInput, false);
          boolean currentValue = buttonInput.getAsBoolean();

          Logger.recordOutput("CustomJoysticks/Joystick Engaged", joystickEngaged);
          Logger.recordOutput("CustomJoysticks/Last Value", lastValue);
          Logger.recordOutput("CustomJoysticks/Current Value", currentValue);

          if (joystickEngaged) {
            toggledState.set(false);
            lastTriggerValues.put(buttonInput, false);
            Logger.recordOutput("CustomJoysticks/Returns", false);
            return false;
          }

          boolean stateChanged = currentValue && !lastValue;
          Logger.recordOutput("CustomJoysticks/State Changed", stateChanged);

          if (stateChanged) {
            toggledState.set(!toggledState.get());
          }

          lastTriggerValues.put(buttonInput, currentValue);
          Logger.recordOutput("CustomJoysticks/Returns", toggledState.get());
          return toggledState.get();
        });
  }
}
