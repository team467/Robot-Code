package frc.lib.mechanisms.flywheels;

import org.wpilib.units.measure.AngularVelocity;

public interface ControlledFlywheel {

  void runAtVelocity(AngularVelocity velocity);

  boolean atSetpoint();

  void stop();
}
