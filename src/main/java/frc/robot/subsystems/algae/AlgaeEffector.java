package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class AlgaeEffector extends SubsystemBase {

  private final AlgaeEffectorIO io;
  private final AlgaeEffectorIOInputsAutoLogged inputs = new AlgaeEffectorIOInputsAutoLogged();
  private final RobotState robotState = RobotState.getInstance();

  public AlgaeEffector(AlgaeEffectorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    robotState.algaeEffectorStowed = inputs.isStowed;
    robotState.algaeEffectorRunning = inputs.removalVolts != 0.0;
    robotState.algaeEffectorExtended = inputs.isFullyExtended;
  }

  public boolean isStowed() {
    return inputs.isStowed;
  }

  public boolean isFullyExtended() {
    return inputs.isFullyExtended;
  }

  public boolean isSpinning() {
    return inputs.isSpinning;
  }

  public Command stowArm() {
    return Commands.run(
            () -> {
              io.setPivotVolts(AlgaeEffectorConstants.RETRACT_VOLTAGE);
              io.setRemovalVolts(AlgaeEffectorConstants.ZERO_VOLTAGE);
            },
            this)
        .until(() -> inputs.isStowed)
        .withTimeout(3)
        .andThen(stop());
  }

  /** When the arm is extended, it starts the algae motor too */
  public Command removeAlgae() {
    return Commands.run(
        () -> {
          if (isFullyExtended() == false) {
            io.setPivotVolts(AlgaeEffectorConstants.EXTEND_VOLTAGE);
          } else {
            io.setPivotVolts(AlgaeEffectorConstants.ZERO_VOLTAGE);
          }
          io.setRemovalVolts(AlgaeEffectorConstants.REMOVAL_VOLTAGE);
        },
        this).until(() -> inputs.pivotPosition >= AlgaeEffectorConstants.REMOVAL_ANGLE);
  }

  /** Stops all algae arm actions */
  public Command stop() {
    return Commands.run(
        () -> {
          io.setPivotVolts(AlgaeEffectorConstants.ZERO_VOLTAGE);
          io.setRemovalVolts(AlgaeEffectorConstants.ZERO_VOLTAGE);
        },
        this);
  }
}
