package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public double intakePercentOutput = 0.0;
    public double extendPercentOutput = 0.0;
    public double intakeVolts = 0.0;
    public double extendVolts = 0.0;
    public double intakeAmps = 0.0;
    public double extendVelocity = 0.0;
    public double extendAmps = 0.0;
    public boolean isCollapsed = false;
    // public boolean manualModeInput;
    public double getExtendPos = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setPercentIntake(double intakePercent) {}

  default void setPercentExtend(double extendPercent) {}

  void setVoltageIntake(double intakeVolts);

  void setVoltageExtend(double extendVolts);

  default void stop() {}

  default void goToPos(double pos) {}

  default boolean isHopperCollapsed() {
    return false;
  }

  default void setPIDEnabled(boolean enabled) {}

  default void resetExtendEncoder() {}
}
