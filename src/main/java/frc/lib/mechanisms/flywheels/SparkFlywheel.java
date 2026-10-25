package frc.lib.mechanisms.flywheels;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

@AutoLog
public class SparkFlywheel implements ReversibleFlywheel, Mechanism {

  private final String subsystemName;
  private final SparkMax spark;
  private final Voltage runVolts;

  public SparkFlywheel(String subsystemName, CANPort canPort, int port, boolean invert,
      Voltage voltage,
      Current currentLimit) {
    this.subsystemName = subsystemName;

    spark = new SparkMax(canPort, port, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(invert)
        .voltageCompensation(12)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit((int) currentLimit.in(Amps));

    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    runVolts = voltage;
  }

  @Override
  public Command run() {
    return run(
        coro -> {
          while (true) {
            spark.setVoltage(runVolts.in(Volts));
            coro.yield();
          }
        }
    ).named(subsystemName + " Run");
  }

  @Override
  public Command reverse() {
    return run(
        coro -> {
          while (true) {
            spark.setVoltage(-runVolts.in(Volts));
            coro.yield();
          }
        }
    ).named(subsystemName + " Reverse");
  }

  @Override
  public Command stop() {
    return run(
        coro -> {
          while (true) {
            spark.setVoltage(Volts.zero());
            coro.yield();
          }
        }
    ).named(subsystemName + " Stop");
  }
}
