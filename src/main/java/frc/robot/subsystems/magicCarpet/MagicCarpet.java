package frc.robot.subsystems.magicCarpet;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class MagicCarpet extends SubsystemBase {
  private final MagicCarpetIO io;
  private final MagicCarpetIOInputsAutoLogged inputs = new MagicCarpetIOInputsAutoLogged();

  /**
   * Initializes the magic carpet
   *
   * @param io The implementation of the hardware
   */
  public MagicCarpet(MagicCarpetIO io) {
    this.io = io;
  }

  /**
   * Runs the magic carpet at a constant speed
   *
   * @return A command to run the magic carpet at a constant speed
   */
  public Command run() {
    return Commands.run(() -> io.setSpeed(MagicCarpetConstants.BELT_SPEED), this)
        .withName("start")
        .finallyDo(this::stop);
  }

  /**
   * Stops the magic carpet
   *
   * @return A command to stop the magic carpet
   */
  public Command stop() {
    return Commands.run(() -> io.setSpeed(0.0), this).withName("stop");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("magicCarpet", inputs);
  }
}
