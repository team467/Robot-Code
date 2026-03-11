package frc.robot.subsystems.magicCarpet;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class MagicCarpet extends SubsystemBase {
  private final MagicCarpetIO io;
  private final MagicCarpetIOInputsAutoLogged inputs = new MagicCarpetIOInputsAutoLogged();

  public MagicCarpet(MagicCarpetIO io) {
    this.io = io;
  }

  public Command run() {
    return Commands.run(() -> io.setSpeed(MagicCarpetConstants.BELT_SPEED), this)
        .withName("start")
        .finallyDo(this::stop);
  }

  public Command stop() {
    return Commands.run(() -> io.setSpeed(0.0), this).withName("stop");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("magicCarpet", inputs);
  }
}
