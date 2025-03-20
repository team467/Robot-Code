package frc.robot.subsystems.fast_algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.algae.AlgaeEffectorConstants;
import frc.robot.subsystems.algae.AlgaeEffectorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class FastAlgaeEffector extends SubsystemBase {

  private final FastAlgaeEffectorIO io;
  // private final FastAlgaeEffectorIOInputsAutoLogged inputs = new AlgaeEffectorIOInputsAutoLogged();
  private final RobotState robotState = RobotState.getInstance(); // TODO: Add to robot state

  public FastAlgaeEffector(FastAlgaeEffectorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    robotState.algaeEffectorStowed = inputs.isStowed;
    robotState.algaeEffectorExtended = inputs.isFullyExtended;
  }

  public boolean isStowed() {
    return inputs.isStowed;
  }

  public boolean isHighPostion() {
    return inputs.isHighPostion;
  }

  public boolean isLowPostion() {
    return inputs.isLowPostion;
  }

  /** Stow Position */
  public Command stowArm() {
    return Commands.run(
            () -> {
              io.setPivotVolts(AlgaeEffectorConstants.RETRACT_VOLTAGE);
            },
            this)
        .until(() -> inputs.isStowed);
  }

  /** High Position */
  public Command removeAlgaeHigh() {
   return Commands.run(
            () -> {
              io.setPivotVolts(FastAlgaeEffectorConstants.HIGH_EXTEND_VOLTAGE);
            },
            this)
       .until(() -> inputs.isHighPostiond);
    
  }

  /** Low position */
  public Command removeAlgaeLow() {
    return Commands.run(
        () -> {
          if (!isLowPostion()) {
            io.setPivotVolts(FastAlgaeEffectorConstants.LOW_EXTEND_VOLTAGE);
          } else {
            io.setPivotVolts(FastAlgaeEffectorConstants.ZERO_VOLTAGE);
          }
        },
        this);
  }

  /** Stops all algae arm actions */
  public Command stop() {
    return Commands.run(
        () -> {
          io.setPivotVolts(FastAlgaeEffectorConstants.ZERO_VOLTAGE);
        },
        this);
  }

  // stow position (with limit switch)
  // high position (hard stop)
  // low position (specific psotion)
  // no removal motor, only pivot
}
