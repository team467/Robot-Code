package frc.robot.subsystems.magicCarpet;

import org.wpilib.command3.Command;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

import frc.lib.SubsystemBase;

public class MagicCarpet extends SubsystemBase {
  private final MagicCarpetIO io;
  private final MagicCarpetIOInputsAutoLogged inputs = new MagicCarpetIOInputsAutoLogged();
  public boolean manualRun;

  /**
   * Initializes the magic carpet
   *
   * @param io The implementation of the hardware
   */
  public MagicCarpet(MagicCarpetIO io) {
    this.io = io;
    this.manualRun = false;
  }

  /**
   * Runs the magic carpet at a constant speed
   *
   * @return A command to run the magic carpet at a constant speed
   */
  public Command run() {
    return this.run(
        coro -> {
          io.setSpeed(MagicCarpetConstants.BELT_SPEED);
          coro.fork(setManualControl(false));
        }
    ).named("MagicCarpet Start");
  }


  /**
   * Control whether to run magic carpet through commands or periodic
   *
   * @param manual Call run manually instead of through periodic
   */
  public Command setManualControl(boolean manual) {
    return this.run(_ -> this.manualRun = manual).named("MagicCarpet SetManualControl");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("magicCarpet", inputs);
    if (!manualRun) {
      if (RobotState.getInstance().indexerRunning) {
        io.setSpeed(MagicCarpetConstants.BELT_SPEED);
      } else {
        io.setSpeed(0.0);
      }
    }
  }
}
