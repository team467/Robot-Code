package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class CustomTriggers {
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;
  private Map<Trigger, Boolean> lastTriggerValues = new HashMap<>();

  public CustomTriggers(
      CommandXboxController driverController, CommandXboxController operatorController) {
    this.operatorController = operatorController;
    this.driverController = driverController;
  }

  public Trigger toggleOnTrueCancelableWithJoystick(
      Trigger buttonInput, DoubleSupplier X, DoubleSupplier Y) {
    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(),
        () -> {
          boolean joystickEngaged =
              Math.abs(X.getAsDouble()) > 0.2 || Math.abs(Y.getAsDouble()) > 0.2;
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
