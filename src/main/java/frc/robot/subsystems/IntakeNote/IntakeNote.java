// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeNote;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

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
    Logger.processInputs("IntakeNote", inputs);
  }

  // Command for intaking a note.
  public Command startIntake() {
    return Commands.run(
        () -> {
          intakeNoteIO.setSpeed(intakeSpeed);
        },
        this);
  }

  // Command for releasing a note.
  public Command release() {
    return Commands.run(
        () -> {
          intakeNoteIO.setSpeed(releaseSpeed);
        },
        this);
  }

  // Command for stopping intake.
  public Command stopIntake() {
    return Commands.run(
        () -> {
          intakeNoteIO.setSpeed(stop);
        },
        this);
  }
}
