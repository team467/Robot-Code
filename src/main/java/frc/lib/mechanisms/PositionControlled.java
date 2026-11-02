package frc.lib.mechanisms;

import org.wpilib.units.measure.Angle;

public interface PositionControlled {
  void goToPosition(Angle position);

  boolean atSetpoint();

  void stop();
}
