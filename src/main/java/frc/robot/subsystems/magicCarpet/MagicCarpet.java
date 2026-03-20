package frc.robot.subsystems.magicCarpet;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.RobotState;

public class MagicCarpet extends SubsystemBase {
  private final MagicCarpetIO io;
  private final MagicCarpetIOInputsAutoLogged inputs = new MagicCarpetIOInputsAutoLogged();
  public boolean manualRun;

  public MagicCarpet(MagicCarpetIO io) {
    this.io = io;
    this.manualRun = false;
  }

  public Command run() {
    return setManualControl(true).andThen(
        Commands.run(() -> io.setSpeed(MagicCarpetConstants.BELT_SPEED), this))
        .finallyDo(() -> setManualControl(false).schedule())
        .withName("start");
  }

  public Command setManualControl(boolean manual) {
    return Commands.runOnce(() -> this.manualRun = manual, this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("magicCarpet", inputs);
    if (!manualRun) {
      if (RobotState.getInstance().indexerRunning) {
        io.setSpeed(MagicCarpetConstants.BELT_SPEED);
      }
      else {
        io.setSpeed(0.0);
      }
    }
  }
    }

