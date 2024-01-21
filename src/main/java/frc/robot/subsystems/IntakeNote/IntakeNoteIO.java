package frc.robot.subsystems.IntakeNote;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeNoteIO {
  @AutoLog
  class IntakeNoteIOInputs {
    // I/p for intake
    public boolean seesNote = false; // Need to communicate with Vision
    public boolean shooterInPosition = false;

    // O/p for intake
    public boolean hasNote = false;

    // Motor IO
    public double appliedVolts = 0.0;
    public double motorVelocity = 0.0;
  }

  default void updateInputs(IntakeNoteIOInputs inputs) {}

  default void setSpeed(double speed) {} // [-1,1], -1 for release, 1 for intake
}
