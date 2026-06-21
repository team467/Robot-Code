package frc.robot.subsystems.intake.extend;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeExtendIO {

  @AutoLog
  class IntakeExtendIOInputs {
    public double extendPercentOutput = 0.0;
    public double extendVolts = 0.0;
    public double extendVelocity = 0.0;
    public double extendAmps = 0.0;
    public boolean isCollapsed = false;
    public boolean atSetpoint = false;
    public boolean hasSetpoint = false;
    public double setpointValue = 0.0;
    public double extendPosition = 0.0;
    public boolean stalledExtended = false;
    public boolean stalledCollapsed = false;
    public double stallExtendTimer = 0.0;
    public double stallCollapseTimer = 0.0;
    public boolean stowed = false;
    public boolean hasPose = false;
  }

  /** Refreshes telemetry from the active extension hardware. */
  default void updateInputs(IntakeExtendIOInputs inputs) {}

  /** Drives the extension by percent output. */
  default void setPercentExtend(double extendPercent) {}

  /** Drives the extension by voltage. */
  default void setVoltageExtend(double extendVolts) {}

  /** Stops the extension if the hardware implementation has a dedicated stop path. */
  default void stop() {}

  /** Requests a closed-loop extension position. */
  default void goToPos(double pos) {}

  /** Enables or disables closed-loop extension control. */
  default void setPIDEnabled(boolean enabled) {}

  /** Sets the extension motor idle behavior. */
  default void setIdleMode(boolean coast) {}

  /** Returns whether closed-loop extension control is currently enabled. */
  default boolean getPIDEnabled() {
    return false;
  }

  /** Resets the extension encoder to the provided position. */
  default void resetExtendEncoder(double position) {}

  /** Returns whether the extension is on the collapsed limit switch. */
  default boolean isCollapsed() {
    return false;
  }

  /** Requests a closed-loop extension position without changing PID enabled state. */
  default void extendToPosition(double position) {}
}
