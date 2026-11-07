package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.StateMachine;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

public interface Shooter {

  StateMachine stateMachine();

  default Time getAirTimeSeconds(Distance distance) {
    return Seconds.of(0.0617 * distance.in(Meters) + 0.872);
  }

}
