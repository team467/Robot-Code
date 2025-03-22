package frc.robot.subsystems.fast_algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class FastAlgaeEffector extends SubsystemBase {

  private final FastAlgaeEffectorIO io;
  private final FastAlgaeEffectorIOInputsAutoLogged inputs =
      new FastAlgaeEffectorIOInputsAutoLogged();
  private final RobotState robotState = RobotState.getInstance(); // TODO: Add to robot state

  public FastAlgaeEffector(FastAlgaeEffectorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    robotState.fastAlgaeEffectorStowed = inputs.isStowed;
    robotState.fastAlgaeEffectorHigh = inputs.isHighPostion;
    robotState.fastAlgaeEffectorLow = inputs.isLowPostion;
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
  // public Command stowArm() {
  //   return Commands.run(
  //       () -> {
  //         io.setPivotVolts(FastAlgaeEffectorConstants.RETRACT_VOLTAGE);
  //       },
  //       this)
  //       .until(() -> inputs.isStowed)
  //       .andThen(() -> io.resetPivotPosition(inputs.pivotPosition));
  // }

  public Command stowArm() {
    return Commands.startEnd(null, null, null);
  }

  /** High Position */
  public Command removeAlgaeHigh() {
    return Commands.run(
            () -> {
              io.setPivotVolts(FastAlgaeEffectorConstants.HIGH_EXTEND_VOLTAGE);
            },
            this)
        .until(() -> inputs.isHighPostion);
  }

  /** Low position */
  public Command removeAlgaeLow() {
    return Commands.run(
            () -> {
              io.setPivotVolts(FastAlgaeEffectorConstants.LOW_EXTEND_VOLTAGE);
            },
            this)
        .until(() -> inputs.isLowPostion);
  }

  /** Stops all algae arm actions */
  public Command stop() {
    return Commands.run(
        () -> {
          io.setPivotVolts(FastAlgaeEffectorConstants.ZERO);
        },
        this);
  }

  // stow position (hard stop)
  // if motor is in reverse but arm is not moving
  // high position (hard stop)
  // low position (specific psotion)
  // no removal motor, only pivot

  // jsut create stowing mech, postions for the other ones, when the velocty is zero and volts are
  // negative,
  // reset motor postion and set to homed (stowed)
  //  start timer when stow starts stop half a second at the end to check for volts
  //
}
