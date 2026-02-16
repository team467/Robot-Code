// Controls the motor
// Reads motor data every 20 ms
// Implements the methods defined in MagicCarpetIO
package frc.robot.subsystems.magiccarpet;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import frc.robot.Schematic;

public class MagicCarpetSparkMax implements MagicCarpetIO {

  private final SparkMax motor; // object controlling motor
  private final RelativeEncoder encoder; // reads motor speed

  public MagicCarpetSparkMax() {

    motor = new SparkMax(Schematic.magicCarpetCanId, MotorType.kBrushless);

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
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.motorVelocity = encoder.getVelocity();
  }

  @Override
  public void setSpeed(double speed) {

    motor.set(MathUtil.clamp(speed, 0.0, 1.0)); // so the is between 0 and 1, 1==100%
  }

  /**
   * implements the stop method from interface, and sets the speed to 0, meaning it immidately stops
   */
  @Override
  public void stop() {
    motor.set(0.0);
  }
}
