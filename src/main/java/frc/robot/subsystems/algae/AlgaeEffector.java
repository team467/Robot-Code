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

  /**
   * Stows the algae effector arm by retracting it and setting the removal voltage to zero.
   *
   * This method sets the pivot voltage to retract the arm and ensures the removal voltage is off.
   * The command will continue until the arm is stowed or a timeout of 3 seconds is reached,
   * after which it will call the stop method.
   *
   * @return A Command that executes the stowing operation.
   */
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

  /**
   * Initiates the algae removal process by adjusting the pivot and removal voltages.
   *
   * If the effector is not fully extended, it sets the pivot voltage to extend it.
   * If it is fully extended, it sets the pivot voltage to zero and activates the removal voltage.
   *
   * @return A Command that executes the algae removal operation.
   */
  public Command removeAlgae() {
    return Commands.run(
        () -> {
          if (!isFullyExtended()) {
            io.setPivotVolts(AlgaeEffectorConstants.EXTEND_VOLTAGE);
          } else {
            io.setPivotVolts(AlgaeEffectorConstants.ZERO_VOLTAGE);
            io.setRemovalVolts(AlgaeEffectorConstants.REMOVAL_VOLTAGE);
          }
        },
        this);
  }

  /**
   * Stops the algae effector by setting the pivot and removal voltages to zero.
   *
   * @return A Command that executes the stop operation.
   */
  public Command stop() {
    return Commands.run(
        () -> {
          io.setPivotVolts(AlgaeEffectorConstants.ZERO_VOLTAGE);
          io.setRemovalVolts(AlgaeEffectorConstants.ZERO_VOLTAGE);
        },
        this);
  }
}
