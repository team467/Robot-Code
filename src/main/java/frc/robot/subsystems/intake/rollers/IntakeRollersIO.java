package frc.robot.subsystems.intake.rollers;

import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
  @AutoLog
  class IntakeIOInputs {
    public double intakePercentOutput = 0.0;
    public double intakeVolts = 0.0;
    public double intakeAmps = 0.0;
    // public boolean manualModeInput;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setPercentIntake(double intakePercent) {}

  default void setVoltageIntake(double intakeVolts) {}

  default void stop() {}
}
