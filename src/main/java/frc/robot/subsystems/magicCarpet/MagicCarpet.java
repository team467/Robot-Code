package frc.robot.subsystems.magicCarpet;

import static frc.robot.subsystems.magicCarpet.MagicCarpetConstants.BELT_VOLTS;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Schematic;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;

@AutoLog
public class MagicCarpet implements Mechanism {

  private final SparkMax spark;

  public MagicCarpet() {
    spark = new SparkMax(CANPort.CAN_D0, Schematic.magicCarpetCanId, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(MagicCarpetConstants.MOTOR_INVERTED)
        .idleMode(IdleMode.kBrake) // stops motor quickly when set to 0
        .smartCurrentLimit((int) MagicCarpetConstants.CURRENT_LIMIT.magnitude());
    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Runs the magic carpet at a constant speed
   *
   * @return A command to run the magic carpet at a constant speed
   */
  public Command run() {
    return this.run(
        coro -> {
          while (true) {
            spark.setVoltage(BELT_VOLTS);
            coro.yield();
          }
        }
    ).named("MagicCarpet Start");
  }

  public Command stop() {
    return run(
        coro -> {
          while (true) {
            spark.setVoltage(Volts.zero());
            coro.yield();
          }
        }).named("MagicCarpet Stop");
  }
}
