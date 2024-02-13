package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged intakeInputs;

  public Intake(IntakeIO intakeIO) {
    super();
    this.intakeIO = intakeIO;
    intakeInputs = new IntakeIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);
  }

  public Command intake(double intakeSpeed) {
    return Commands.run(
        () -> {
          Logger.recordOutput("Intake/DesiredSpeed", intakeSpeed);
          intakeIO.setSpeed(intakeSpeed);
        },
        this);
  }

  public Command release(double releaseSpeed) {
    return Commands.run(
        () -> {
          Logger.recordOutput("Intake/DesiredSpeed", releaseSpeed);
          intakeIO.setSpeed(releaseSpeed);
        },
        this);
  }

  public Command stop() {
    return Commands.run(
        () -> {
          Logger.recordOutput("Intake/DesiredSpeed", IntakeConstants.STOP_SPEED);
          intakeIO.setSpeed(IntakeConstants.STOP_SPEED);
        },
        this);
  }
}
