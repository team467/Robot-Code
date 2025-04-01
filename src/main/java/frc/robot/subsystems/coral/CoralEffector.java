package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.ElevatorPosition;
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
    return Commands.runOnce(
        () -> {
          io.setSpeed(0);
        },
        this);
  }

  public Command dumpCoral() {
    return Commands.run(
            () -> {
              if (RobotState.getInstance().elevatorPosition == ElevatorPosition.L4) {
                io.setSpeed(CoralEffectorConstants.L4_CORAL_SPEED_OUT.get());
              } else if (RobotState.getInstance().elevatorPosition == ElevatorPosition.L1) {
                io.setSpeed(CoralEffectorConstants.L1_CORAL_SPEED_OUT.get());
              } else {
                io.setSpeed(CoralEffectorConstants.CORAL_SPEED_OUT.get());
              }
            },
            this)
        .onlyWhile(this::hasCoral)
        .onlyIf(this::hasCoral);
  }

  public Command dumpCoral(double speed) {
    return Commands.run(
            () -> {
              io.setSpeed(speed);
            },
            this)
        .onlyWhile(this::hasCoral)
        .onlyIf(this::hasCoral);
  }

  public Command takeBackCoral() {
    return Commands.run(
            () -> {
              io.setSpeed(CoralEffectorConstants.CORAL_RETAKE_SPEED.get());
            },
            this)
        .withTimeout(CoralEffectorConstants.EFFECTOR_PULLBACK_SECONDS.get());
  }

  public Command intakeCoral() {
    return Commands.run(
            () -> {
              io.setSpeed(CoralEffectorConstants.CORAL_INTAKE_SPEED.get());
            },
            this)
        .until(this::hasCoral);
    //        .finallyDo(this::takeBackCoral)
    //        .withTimeout(CoralEffectorConstants.EFFECTOR_PULLBACK_SECONDS.get());
  }
}
