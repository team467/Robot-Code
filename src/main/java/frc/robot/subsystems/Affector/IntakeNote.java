// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Affector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeNote extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeNoteIO intakeNoteIO;

  // Intializes States
  private enum State {
    DISABLED,
    INTAKE,
    RELEASE,
    STOP
  }

  private State state;
  
    // Initializes Wants
    private enum Wants {
      NOTE,
      NOTHING
    }
  
    private Wants wants = Wants.NOTHING;
    private boolean hasNote = false;

  // Intializes IntakeNote
  public IntakeNote(IntakeNoteIO intakeNoteIO) {
    super();
    this.intakeNoteIO = intakeNoteIO;
    state = State.STOP;
  }

  public void setWants(Wants wants)
  {
    this.wants = wants;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeNoteIO.updateIO(null); // Need to provide "inputs" and need an AutoLogged class for IntakeNote
  }

  public void intake() {
    
  }

}
