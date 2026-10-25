package frc.lib.mechanisms.flywheels;

import org.wpilib.command3.Command;

public interface ReversibleFlywheel extends Flywheel {

  Command reverse();
}
