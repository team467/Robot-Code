package frc.robot.subsystems.shooter;

import org.wpilib.command3.Command;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;

public interface Shooter {

  Command shoot(Distance distance);

  Command shoot(AngularVelocity velocity);

  Command stop();

  boolean atSetpoint();
}
