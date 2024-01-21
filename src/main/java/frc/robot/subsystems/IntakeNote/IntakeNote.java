// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeNote;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeNote extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeNoteIO intakeNoteIO;

  private boolean hasNote = false;

  // Intializes IntakeNote
  public IntakeNote(IntakeNoteIO intakeNoteIO) {
    super();
    this.intakeNoteIO = intakeNoteIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeNoteIO.updateInputs(null); // Need to provide "inputs" and need an AutoLogged class for IntakeNote
  }

  public void startIntake() {
    hasNote = false;
    intakeNoteIO.setSpeed(0);
  }

  // Command for intaking a note.
  /* public Command IntakeCommand()
  {
    return intakeNoteIO.setSpeed(0);
  } */

  public void release() {
    hasNote = true;
    intakeNoteIO.setSpeed(0);
    ;
  }

  public void stop() {
    intakeNoteIO.setSpeed(0);
  }
}
