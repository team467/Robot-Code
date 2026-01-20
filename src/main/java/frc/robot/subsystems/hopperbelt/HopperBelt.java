package frc.robot.subsystems.hopperbelt;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopperbelt.HopperBeltIOInputsAutoLogged;

public class HopperBelt extends SubsystemBase {
  private final HopperBeltIO io;
  private final HopperBeltIOInputsAutoLogged inputs = new HopperBeltIOInputsAutoLogged();

  public HopperBelt(HopperBeltIO io) {
    this.io = io;
  }

  public Command start() {
    return Commands.run(() -> io.setSpeed(HopperBeltConstants.BELT_SPEED), this);
  }

  public Command stop() {
    return Commands.run(() -> io.stop(), this);
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
  }
}
