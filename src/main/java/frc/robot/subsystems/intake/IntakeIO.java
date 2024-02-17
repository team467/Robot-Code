package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeNoteIOInputs {
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double motorCurrentAmps = 0.0;
  }

  default void updateInputs(IntakeNoteIOInputs intakeInputs) {}

  default void setSpeed(double speed) {}
}
