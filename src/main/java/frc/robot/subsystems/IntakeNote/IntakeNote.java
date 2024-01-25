// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeNote;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeNote extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeNoteIO intakeNoteIO;

  private final IntakeNoteIOInputsAutoLogged inputs;
  private boolean hasNote = false;
  private final double intakeSpeed = 0.3;
  private final double releaseSpeed = -0.3;
  private final double stop = 0.0;

  // Intializes IntakeNote
  public IntakeNote(IntakeNoteIO intakeNoteIO) {
    super();
    this.intakeNoteIO = intakeNoteIO;
    inputs = new IntakeNoteIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeNoteIO.updateInputs(inputs);
  }

  /*public void startIntake() {
    hasNote = false;
    intakeNoteIO.setSpeed(intakeSpeed);
  }*/

  // Command for intaking a note.
  public Command startIntake() {
    return Commands.run(() -> {intakeNoteIO.setSpeed(intakeSpeed);}, this);
  }

  /*public void release() {
    hasNote = true;
    intakeNoteIO.setSpeed(releaseSpeed);
  }*/

  // Command for releasing a note.
  public Command release() {
    return Commands.run(() -> {intakeNoteIO.setSpeed(releaseSpeed);}, this);
  }

  /*public void stop() {
    intakeNoteIO.setSpeed(stop);
  }*/

  // Command for stopping intake.
  public Command stop() {
    return Commands.run(() -> {intakeNoteIO.setSpeed(stop);}, this);
  }
}
