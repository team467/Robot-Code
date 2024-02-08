package frc.robot.subsystems.IntakeNote;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeNoteIO {
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
