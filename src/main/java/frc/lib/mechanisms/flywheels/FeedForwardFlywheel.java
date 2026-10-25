package frc.lib.mechanisms.flywheels;

import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.command3.Coroutine;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

@AutoLog
abstract public class FeedForwardFlywheel implements ControlledFlywheel {

  private final SimpleMotorFeedforward ff;
  private AngularVelocity setpoint;

  protected FeedForwardFlywheel(double ks, double kv, double ka) {
    ff = new SimpleMotorFeedforward(ks, kv, ka);
  }

  @Override
  public boolean atSetpoint() {
    return getVelocity().equals(setpoint);
  }

  protected Consumer<Coroutine> stopCoro() {
    return _ -> setVoltage(Volts.zero());
  }

  protected Consumer<Coroutine> runAtVelocityCoro(AngularVelocity setpoint) {
    return coro -> {
      while (true) {
        this.setpoint = setpoint;

        setVoltage(
            Volts.of(
                ff.calculate(
                    getVelocity().in(RadiansPerSecond),
                    setpoint.in(RadiansPerSecond)
                )
            )
        );
        coro.yield();
      }
    };
  }

  protected abstract void setVoltage(Voltage voltage);

  protected abstract AngularVelocity getVelocity();
}
