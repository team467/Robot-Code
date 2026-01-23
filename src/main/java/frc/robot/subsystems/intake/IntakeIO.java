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

    public boolean isSlipping = false;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setPercentIntake(double intakePercent) {}

  default void setPercentExtend(double extendPercent) {}

  void setVoltageIntake(double intakeVolts);

  void setVoltageExtend(double extendVolts);

  default void stop() {}

  default void goToSetpoint() {}

  default boolean isHopperCollapsed() {
    return false;
  }

  default boolean slipCheck() {
    return false;
  }
}
