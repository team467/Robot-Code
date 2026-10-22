package first.robot.mechanisms.magicCarpet;

import static first.robot.mechanisms.magicCarpet.MagicCarpetConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import first.robot.Schematic;

public class MagicCarpetIOSparkMax implements MagicCarpetIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;

  public MagicCarpetIOSparkMax() {
    motor = new SparkMax(CAN_PORT, Schematic.magicCarpetCanId, MotorType.kBrushless);

    var config = new SparkMaxConfig();
    config
        .inverted(MOTOR_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CURRENT_LIMIT);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(MagicCarpetIOInputs inputs) {
    var appliedOutput = motor.getAppliedOutput();
    var busVoltage = motor.getBusVoltage();
    var current = motor.getOutputCurrent();
    var velocity = encoder.getVelocity();
    inputs.connected =
        appliedOutput.isValid() && busVoltage.isValid() && current.isValid() && velocity.isValid();
    inputs.appliedVolts = appliedOutput.get() * busVoltage.get();
    inputs.currentAmps = current.get();
    inputs.motorVelocity = velocity.get();
  }

  @Override
  public void applyOutputs(MagicCarpetIOOutputs outputs) {
    switch (outputs.mode) {
      case DUTY_CYCLE ->
          motor.setVoltage(Math.clamp(outputs.targetSpeed, 0.0, 1.0) * 12.0);
      case STOP -> motor.stopMotor();
    }
  }
}
