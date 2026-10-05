package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
  @AutoLog
  class IntakeRollersIOInputs {
    public double intakePercentOutput = 0.0;
    public double intakeVolts = 0.0;
    public double intakeAmps = 0.0;
    public double intakeRPM = 0.0;
  }

  /** Refreshes telemetry from the active roller hardware. */
  default void updateInputs(IntakeRollersIOInputs inputs) {}

  /** Drives the rollers by percent output. */
  default void setPercentIntake(double intakePercent) {}

  /** Drives the rollers by voltage. */
  default void setVoltageIntake(double intakeVolts) {}

  /** Stops the rollers if the hardware implementation has a dedicated stop path. */
  default void stop() {}
}
