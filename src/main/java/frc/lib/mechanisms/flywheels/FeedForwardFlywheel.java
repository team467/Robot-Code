package frc.lib.mechanisms.flywheels;

import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

@AutoLog
public class FeedForwardFlywheel implements ControlledFlywheel, Mechanism {

  private final String subsystemName;
  private final SimpleMotorFeedforward ff;
  private final Consumer<Voltage> setVoltage;
  private final Supplier<AngularVelocity> getVelocity;
  private AngularVelocity setpoint;

  public FeedForwardFlywheel(String subsystemName, double ks, double kv, double ka,
      Consumer<Voltage> setVoltage, Supplier<AngularVelocity> getVelocity) {
    this.subsystemName = subsystemName;
    ff = new SimpleMotorFeedforward(ks, kv, ka);

    this.setVoltage = setVoltage;
    this.getVelocity = getVelocity;
  }

  @Override
  public boolean atSetpoint() {
    return getVelocity.get().equals(setpoint);
  }

  @Override
  public Command stop() {
    return run(
        coro -> {
          while (true) {
            setVoltage.accept(Volts.zero());
            coro.yield();
          }
        }
    ).named(subsystemName + " Stop");
  }

  @Override
  public Command runAtVelocity(AngularVelocity setpoint) {
    return run(coro -> {
      this.setpoint = setpoint;

      while (!getVelocity.get().equals(setpoint)) {
        setVoltage.accept(
            Volts.of(
                ff.calculate(
                    getVelocity.get().in(RadiansPerSecond),
                    setpoint.in(RadiansPerSecond)
                )
            )
        );
        coro.yield();
      }
    }).named(subsystemName + "Run At Velocity");
  }
}
