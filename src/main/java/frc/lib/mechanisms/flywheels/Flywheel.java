package frc.lib.mechanisms.flywheels;

import org.wpilib.command3.Command;

public interface Flywheel {

  Command run();

  Command stop();
}
