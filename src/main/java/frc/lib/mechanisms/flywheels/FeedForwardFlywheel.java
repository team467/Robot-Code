package frc.lib.mechanisms.flywheels;

import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

@AutoLog
public class FeedForwardFlywheel implements ControlledFlywheel, Mechanism {

  private final SimpleMotorFeedforward ff;
  private final Consumer<Voltage> setVoltage;
  private final Supplier<AngularVelocity> getVelocity;
  private AngularVelocity setpoint;

  public FeedForwardFlywheel(double ks, double kv, double ka,
      Consumer<Voltage> setVoltage, Supplier<AngularVelocity> getVelocity) {
    ff = new SimpleMotorFeedforward(ks, kv, ka);

    this.setVoltage = setVoltage;
    this.getVelocity = getVelocity;
  }

  @Override
  public boolean atSetpoint() {
    return getVelocity.get().equals(setpoint);
  }

  @Override
  public void stop() {
    setVoltage.accept(Volts.zero());
  }

  @Override
  public void runAtVelocity(AngularVelocity setpoint) {
    this.setpoint = setpoint;

    setVoltage.accept(
        Volts.of(
            ff.calculate(
                getVelocity.get().in(RadiansPerSecond),
                setpoint.in(RadiansPerSecond)
            )
        )
    );
  }
}
