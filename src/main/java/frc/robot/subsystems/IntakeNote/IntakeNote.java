// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeNote;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class IntakeNote extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeNoteIO intakeNoteIO;

  private final IntakeNoteIOInputsAutoLogged intakeInputs;
  private final double intakeSpeed = 0.2;
  private final double releaseSpeed = -0.2;
  private final double stop = 0.0;
  private final BooleanSupplier hasNote = () -> false;

  // Intializes IntakeNote
  public IntakeNote(IntakeNoteIO intakeNoteIO) {
    super();
    this.intakeNoteIO = intakeNoteIO;
    intakeInputs = new IntakeNoteIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeNoteIO.updateInputs(intakeInputs);
    Logger.processInputs("IntakeNote", intakeInputs);
  }

  // Command for intaking a note.
  public Command startIntake() {
    return Commands.run(
        () -> {
          intakeNoteIO.setSpeed(intakeSpeed);
        },
        this).until(() -> hasNote.getAsBoolean());
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
  public Command stop() {
    return Commands.run(
        () -> {
          intakeNoteIO.setSpeed(stop);
        },
        this);
  }
}
