package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
  @AutoLog
  class IntakeRollersIOInputs {
    public double intakePercentOutput = 0.0;
    public double intakeVolts = 0.0;
    public double intakeAmps = 0.0;
    // public boolean manualModeInput;
  }

  default void updateInputs(IntakeRollersIOInputs inputs) {}

  default void setPercentIntake(double intakePercent) {}

  default void setVoltageIntake(double intakeVolts) {}

  default void stop() {}
}
