package frc.lib.mechanisms.flywheels;

import static org.wpilib.units.Units.Minute;
import static org.wpilib.units.Units.Revolution;
import static org.wpilib.units.Units.Volts;

import java.util.function.Consumer;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

abstract public class PIDFlywheel implements ControlledFlywheel, Mechanism {

  private final PIDController pid;

  protected PIDFlywheel(double kp, double kd, double ki) {
    pid = new PIDController(kp, ki, kd);
  }

  protected Consumer<Coroutine> stopCoro() {
    return _ -> setVoltage(Volts.zero());
  }

  protected Consumer<Coroutine> runAtVelocityCoro(AngularVelocity setpoint) {
    return _ -> setVoltage(
        Volts.of(
            pid.calculate(
                getVelocity().in(Revolution.per(Minute)),
                setpoint.in(Revolution.per(Minute))
            )
        )
    );
  }

  abstract protected void setVoltage(Voltage volts);

  abstract protected AngularVelocity getVelocity();
}
