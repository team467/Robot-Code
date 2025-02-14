package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class CustomTriggers {
  private boolean lastFaceReefButtonInput = false;
  private boolean aligningToFaceReef = false;
  private boolean lastAlignToLeftBranchInput = false;
  private boolean aligningToLeftBranch = false;

  private boolean aligningToRightBranch = false;
  private boolean lastAlignToRightBranchInput = false;

  public Trigger faceReef(
      Trigger alignButton, DoubleSupplier turnJoystickX, DoubleSupplier turnJoystickY) {
    return new Trigger(
        () -> {
          boolean turnJoystickEngaged =
              (Math.abs(turnJoystickX.getAsDouble()) > 0.2
                  || Math.abs(turnJoystickY.getAsDouble()) > 0.2);
          if (!turnJoystickEngaged) {
            if (alignButton.getAsBoolean()) {
              aligningToFaceReef = !aligningToFaceReef;
            }
          } else {
            aligningToFaceReef = false;
          }
          lastFaceReefButtonInput = alignButton.getAsBoolean() || !turnJoystickEngaged;
          return aligningToFaceReef;
        });
  }

  public Trigger alignToReefLeft(
      Trigger alignButton, DoubleSupplier driveJoystickX, DoubleSupplier driveJoystickY) {
    return new Trigger(
        () -> {
          boolean turnJoystickEngaged =
              (Math.abs(driveJoystickX.getAsDouble()) > 0.2
                  || Math.abs(driveJoystickY.getAsDouble()) > 0.2);
          if (!turnJoystickEngaged) {
            if (alignButton.getAsBoolean()) {
              aligningToLeftBranch = !aligningToLeftBranch;
            }
          } else {
            aligningToLeftBranch = false;
          }
          lastAlignToLeftBranchInput = alignButton.getAsBoolean() || !turnJoystickEngaged;
          return aligningToLeftBranch;
        });
  }

  public Trigger alignToReefRight(
      Trigger alignButton, DoubleSupplier driveJoystickX, DoubleSupplier driveJoystickY) {
    return new Trigger(
        () -> {
          boolean turnJoystickEngaged =
              (Math.abs(driveJoystickX.getAsDouble()) > 0.2
                  || Math.abs(driveJoystickY.getAsDouble()) > 0.2);
          if (!turnJoystickEngaged) {
            if (alignButton.getAsBoolean()) {
              aligningToRightBranch = !aligningToRightBranch;
            }
          } else {
            aligningToRightBranch = false;
          }
          lastAlignToRightBranchInput = alignButton.getAsBoolean() || !turnJoystickEngaged;
          return aligningToRightBranch;
        });
  }
}
