// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeNote;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeNote extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeNoteIO intakeNoteIO;

  // Intializes States
  private enum State {
    DISABLED,
  }

  private State state;

  private boolean hasNote = false;

  // Intializes IntakeNote
  public IntakeNote(IntakeNoteIO intakeNoteIO) {
    super();
    this.intakeNoteIO = intakeNoteIO;
  }

  /* TODO: Get rid of State stuff in here, just have methods like startIntake that are used by the actual commands.*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeNoteIO.updateInputs(
        null); // Need to provide "inputs" and need an AutoLogged class for IntakeNote

    switch (state) {
      case DISABLED -> intakeNoteIO.setPercent(0.0);
    }
  }

  public void startIntake() {
    hasNote = false;
    intakeNoteIO.setPercent(-1.0);
  }

  public void release() {
    hasNote = true;
    intakeNoteIO.setPercent((1.0));
  }

  public void stop() {
    intakeNoteIO.setPercent(0.0);
  }
}
