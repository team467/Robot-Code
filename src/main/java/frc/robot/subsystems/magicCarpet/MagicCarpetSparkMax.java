// Controls the motor
// Reads motor data every 20 ms
// Implements the methods defined in MagicCarpetIO
package frc.robot.subsystems.magicCarpet;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.util.MathUtil;
import frc.robot.Schematic;

public class MagicCarpetSparkMax implements MagicCarpetIO {

  private final SparkMax motor; // object controlling motor
  private final RelativeEncoder encoder; // reads motor speed

  public MagicCarpetSparkMax() {

    motor = new SparkMax(CANPort.CAN_D0, Schematic.magicCarpetCanId, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(MagicCarpetConstants.MOTOR_INVERTED)
        .idleMode(IdleMode.kBrake) // stops motor quickly when set to 0
        .smartCurrentLimit(MagicCarpetConstants.CURRENT_LIMIT);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // to measure speed
    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(MagicCarpetIOInputs inputs) {
    // Called every 20 ms by subsystem periodic
    inputs.appliedVolts = motor.getAppliedOutput().get() * motor.getBusVoltage().get();
    inputs.currentAmps = motor.getOutputCurrent().get();
    inputs.motorVelocity = encoder.getVelocity().get();
  }

  @Override
  public void setSpeed(double speed) {
    motor.setVoltage(MagicCarpetConstants.CURRENT_LIMIT * Math.clamp(speed, 0.0, 1.0));
  }

  /**
   * implements the stop method from interface, and sets the speed to 0, meaning it immidately stops
   */
  @Override
  public void stop() {
    motor.setVoltage(0);
  }
}
