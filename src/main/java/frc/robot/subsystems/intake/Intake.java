package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged intakeInputs;
  private final RobotState robotState = RobotState.getInstance();

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

  public Command intake() {
    return Commands.run(
            () -> {
              Logger.recordOutput("Intake/DesiredSpeed", IntakeConstants.INTAKE_SPEED.get());
              intakeIO.setSpeed(IntakeConstants.INTAKE_SPEED.get());
              robotState.intaking = true;
            },
            this)
        .finallyDo(() -> robotState.intaking = false);
  }

  public Command release() {
    return Commands.run(
        () -> {
          Logger.recordOutput("Intake/DesiredSpeed", IntakeConstants.RELEASE_SPEED);
          intakeIO.setSpeed(IntakeConstants.RELEASE_SPEED);
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
