package frc.robot.subsystems.shooter;

import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;

public interface Shooter {

  void shoot(Distance distance);

  void shoot(AngularVelocity velocity);

  void stop();

  boolean atSetpoint();
}
