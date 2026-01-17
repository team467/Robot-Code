package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public double intakePercentOutput = 0.0;
    public double extendPercentOutput = 0.0;
    public double intakeVolts = 0.0;
    public double extendedVolts = 0.0;
    public double intakeAmps = 0.0;
    public double extendAmps = 0.0;
    public boolean isExtended = false;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setPercent(double intakePercent, double extendPercent) {}

  default void setVoltage(double intakeVolts, double extendVolts) {}

  default void stop() {}

  default boolean isHopperExtended() {
    return false;
  }
}
