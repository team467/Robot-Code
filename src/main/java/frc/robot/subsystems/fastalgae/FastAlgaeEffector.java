package frc.robot.subsystems.fastalgae;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class FastAlgaeEffector extends SubsystemBase {
  public static Timer stowTimer = new Timer();
  private boolean stowed = false;
  private final FastAlgaeEffectorIO io;
  private final FastAlgaeEffectorIOInputsAutoLogged inputs =
      new FastAlgaeEffectorIOInputsAutoLogged();
  private final RobotState robotState = RobotState.getInstance();

  public FastAlgaeEffector(FastAlgaeEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    robotState.fastAlgaeEffectorHigh = inputs.isHighPostion;
    robotState.fastAlgaeEffectorLow = inputs.isLowPostion;
    robotState.fastAlgaeStowed = stowed;

    if (inputs.pivotVolts > 0.1 && stowed) {
      stowed = false;
    }

    if (isSlipping()) {
      stowTimer.start();
    } else {
      stowTimer.stop();
      stowTimer.reset();
    }

    if (stowTimer.get() > 0.05) {
      stowed = true;
      io.resetPivotPosition(FastAlgaeEffectorConstants.STOW_ANGLE);
      stowTimer.stop();
      stowTimer.reset();
    }
    if (stowed) {
      io.resetPivotPosition(FastAlgaeEffectorConstants.STOW_ANGLE);
    }

    inputs.isStowed = stowed;
    robotState.fastAlgaeEffectorStowed = inputs.isStowed;
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
    return Commands.run(() -> io.setPivotVolts(FastAlgaeEffectorConstants.RETRACT_VOLTAGE), this)
        .until(() -> stowed)
        .andThen(stop());
  }

  /** High Position */
  public Command removeAlgaeHigh() {
    return Commands.run(
            () -> {
              io.setPivotVolts(FastAlgaeEffectorConstants.HIGH_EXTEND_VOLTAGE);
            },
            this)
        .until(() -> inputs.isHighPostion)
        .andThen(stowArm());
  }

  /** Low position */
  public Command removeAlgaeLow() {
    return Commands.run(
            () -> {
              io.setPivotVolts(FastAlgaeEffectorConstants.LOW_EXTEND_VOLTAGE);
            },
            this)
        .until(() -> inputs.isLowPostion)
        .andThen(stowArm());
  }

  /** Stops all algae arm actions */
  public Command stop() {
    return Commands.run(
        () -> {
          io.setPivotVolts(FastAlgaeEffectorConstants.ZERO);
        },
        this);
  }

  public boolean isSlipping() {
    return Math.abs(inputs.pivotVelocity) < 0.1 && inputs.pivotVolts < -0.1;
  }
  // stow position (hard stop)
  // if motor is in reverse but arm is not moving
  // if motor is in reverse but arm is not moving
  // high position (hard stop)
  // low position (specific psotion)
  // no removal motor, only pivot

  // jsut create stowing mech, postions for the other ones, when the velocty is zero and volts are
  // negative,
  // reset motor postion and set to homed (stowed)
  //  start timer when stow starts stop half a second at the end to check for volts
}
