package frc.lib.mechanisms.flywheels;

import static frc.lib.utils.PhoenixUtil.tryUntilOk;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

@AutoLog
public class KrakenFlywheel implements ReversibleFlywheel, Mechanism {

  private final TalonFX kraken;

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  private final Voltage volts;
  private final String subsystemName;

  public KrakenFlywheel(String subsystemName, CANPort can, int port, InvertedValue invertedValue,
      Current currentLimit, Voltage volts) {
    this.subsystemName = subsystemName;
    this.volts = volts;

    kraken = new TalonFX(port, CANBus.systemcore(
        switch (can) {
          case CAN_D0 -> 0;
          case CAN_D1 -> 1;
          case CAN_D2 -> 2;
          case CAN_D3 -> 3;
          case CAN_D4 -> 4;
          default -> throw new RuntimeException("Invalid CAN port");
        }
    ));

    var config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast).withInverted(invertedValue);
    config.CurrentLimits.withStatorCurrentLimit(currentLimit).withStatorCurrentLimitEnable(true);

    tryUntilOk(5, () -> kraken.getConfigurator().apply(config));

    appliedVolts = kraken.getMotorVoltage();
    current = kraken.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, appliedVolts, current);
  }

  @Override
  public Command reverse() {
    return run(
        coro -> {
          while (true) {
            kraken.setVoltage(volts.times(-1));
            coro.yield();
          }
        }
    ).named(subsystemName + " Reverse");
  }

  @Override
  public Command run() {
    return run(
        coro -> {
          while (true) {
            kraken.setVoltage(volts);
            coro.yield();
          }
        }
    ).named(subsystemName + " Run");
  }

  @Override
  public Command stop() {
    return run(
        coro -> {
          while (true) {
            kraken.setVoltage(Volts.zero());
            coro.yield();
          }
        }
    ).named(subsystemName + " Stop");
  }
}
