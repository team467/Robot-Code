package frc.robot.subsystems.kraken;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kraken extends SubsystemBase {

  public KrakenIO io;

  public Kraken(KrakenIO io) {
    this.io = io;
  }

  public Command setVoltage(double volts) {
    return Commands.run(() -> io.setVoltage(volts));
  }
}
