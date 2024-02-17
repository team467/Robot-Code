package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeNoteIO intakeNoteIO;
  private final IntakeNoteIOInputsAutoLogged intakeInputs;

  public Intake(IntakeNoteIO intakeNoteIO) {
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

  public Command intake() {
    return Commands.run(
        () -> {
          Logger.recordOutput("IntakeNote/DesiredSpeed", IntakeConstants.INTAKE_SPEED);
          intakeNoteIO.setSpeed(IntakeConstants.INTAKE_SPEED);
        },
        this);
  }

  public Command release() {
    return Commands.run(
        () -> {
          Logger.recordOutput("IntakeNote/DesiredSpeed", IntakeConstants.RELEASE_SPEED);
          intakeNoteIO.setSpeed(IntakeConstants.RELEASE_SPEED);
        },
        this);
  }

  public Command stop() {
    return Commands.run(
        () -> {
          Logger.recordOutput("IntakeNote/DesiredSpeed", IntakeConstants.STOP_SPEED);
          intakeNoteIO.setSpeed(IntakeConstants.STOP_SPEED);
        },
        this);
  }
}
