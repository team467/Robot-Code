// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeNote;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeNote extends SubsystemBase {
  private final IntakeNoteIO intakeNoteIO;

  private final IntakeNoteIOInputsAutoLogged intakeInputs;
  // TODO: Change these speeds as needed once we test.
  private final double intakeSpeed = 0.2;
  private final double releaseSpeed = -0.2;
  private final double stop = 0.0;

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

  public Command startIntake() {
    return Commands.run(
        () -> {
          Logger.recordOutput("IntakeNote/DesiredSpeed", intakeSpeed);
          intakeNoteIO.setSpeed(intakeSpeed);
        },
        this);
  }

  public Command release() {
    return Commands.run(
        () -> {
          intakeNoteIO.setSpeed(releaseSpeed);
        },
        this);
  }

  public Command stop() {
    return Commands.run(
        () -> {
          intakeNoteIO.setSpeed(stop);
        },
        this);
  }
}
