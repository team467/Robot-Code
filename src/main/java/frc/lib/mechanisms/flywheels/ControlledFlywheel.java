package frc.lib.mechanisms.flywheels;

import org.wpilib.command3.Command;
import org.wpilib.units.measure.AngularVelocity;

public interface ControlledFlywheel {

  Command runAtVelocity(AngularVelocity velocity);

  boolean atSetpoint();

  Command stop();
}
