package frc.robot.subsystems.intake.extend;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeExtendIO {

  default void updateInputs(IntakeExtendIOInputs inputs) {
  }

  default void setVoltageExtend(double extendVolts) {
  }

  default void goToPos(double pos) {
  }

  default void setPIDEnabled(boolean enabled) {
  }

  default void resetExtendEncoder(double position) {
  }

  @AutoLog
  class IntakeExtendIOInputs {

    public double extendPercentOutput = 0.0;
    public double extendVolts = 0.0;
    public double extendVelocity = 0.0;
    public double extendAmps = 0.0;
    public boolean isCollapsed = false;
    // public boolean manualModeInput;
    public boolean atSetpoint = false;
    public boolean hasSetpoint = false;
    public double setpointValue = 0.0;
    public double getExtendPos = 0.0;
    public boolean stalledExtended = false;
    public boolean stalledCollapsed = false;
    public double stallExtendTimer = 0.0;
    public double stallCollapseTimer = 0.0;
    public boolean stowed = false;
    public boolean hasPose = false;
  }
}
