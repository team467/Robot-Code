package frc.robot.subsystems.HopperBelt;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperBelt extends SubsystemBase {
  private final HopperBeltIO io;
  private final HopperBeltIOInputsAutoLogged inputs = new HopperBeltIOInputsAutoLogged();

  public HopperBelt(HopperBeltIO io) {
    this.io = io;
  }

  public Command start() {
    return Commands.run(() -> io.setSpeed(HopperBeltConstants.BELT_SPEED), this);
  }

  // made a new command that returns the method in the sparkMax which starts the motor
  public Command stop() {
    return Commands.run(() -> io.stop(), this);
  }

  // made a new command that returns the method in the sparkMax which stops the motor

  @Override
  public void periodic() {

    io.updateInputs(inputs);
  }

  // updates inputs every 20 milliseconds, io object

}
