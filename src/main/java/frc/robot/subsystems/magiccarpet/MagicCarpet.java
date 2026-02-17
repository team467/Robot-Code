package frc.robot.subsystems.magiccarpet;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MagicCarpet extends SubsystemBase {
  private final MagicCarpetIO io;
  private final MagicCarpetIOInputsAutoLogged inputs = new MagicCarpetIOInputsAutoLogged();

  public MagicCarpet(MagicCarpetIO io) {
    this.io = io;
  }

  public Command start() {
    return Commands.run(() -> io.setSpeed(MagicCarpetConstants.BELT_SPEED), this);
  }

  public Command stop() {
    return Commands.run(() -> io.stop(), this);
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
  }
}
