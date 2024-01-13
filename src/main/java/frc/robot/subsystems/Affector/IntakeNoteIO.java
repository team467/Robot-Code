package frc.robot.subsystems.Affector;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeNoteIO 
{
  @AutoLog
  class IntakeNoteIOInputs {
    // IO for grabber
    public boolean hasNote = false;
    public boolean seesNote = false; // Need to communicate with Vision
    public double appliedVolts = 0.0;
    public double motorVelocity = 0.0;
    // Need to add more IO's
  }
  
  default void updateIO(IntakeNoteIOInputs inputs) {}
}
