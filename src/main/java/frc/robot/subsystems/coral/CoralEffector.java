package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class CoralEffector extends SubsystemBase {

  private CoralEffectorIO io;
  private CoralEffectorIOInputsAutoLogged inputs = new CoralEffectorIOInputsAutoLogged();
  private final RobotState robotState = RobotState.getInstance();

  public CoralEffector(CoralEffectorIO io) {
    this.io = io;
    this.inputs = new CoralEffectorIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralEffector", inputs);
    robotState.intakingCoral = (inputs.appliedVolts != 0 && !inputs.hasCoral);
    robotState.hasCoral = inputs.hasCoral;
    robotState.hopperSeesCoral = inputs.hopperSeesCoral;
    robotState.dumpingCoral = (inputs.appliedVolts != 0 && inputs.hopperSeesCoral);
  }

  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  public boolean hopperSeesCoral() {

    return inputs.hopperSeesCoral;
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setSpeed(0);
        },
        this);
  }

  public Command dumpCoral() {
    return Commands.run(
            () -> {
              io.setSpeed(CoralEffectorConstants.CORAL_SPEED_OUT.get());
            },
            this)
        .until(() -> !this.hasCoral());
  }

  public Command takeBackCoral() {
    return Commands.run(
        () -> {
          io.setSpeed(CoralEffectorConstants.CORAL_RETAKE_SPEED.get());
        },
        this);
  }

  public Command intakeCoral() {
    return Commands.run(
            () -> {
              io.setSpeed(CoralEffectorConstants.CORAL_INTAKE_SPEED.get());
            },
            this)
        .until(this::hasCoral)
        .finallyDo(this::takeBackCoral)
        .withTimeout(CoralEffectorConstants.EFFECTOR_PULLBACK_SECONDS.get());
  }
}
