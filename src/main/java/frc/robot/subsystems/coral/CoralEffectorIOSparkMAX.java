package frc.robot.subsystems.coral;

import static frc.lib.utils.SparkUtil.tryUntilOk;
import static frc.robot.Schematic.coralHaveCoralDioId;
import static frc.robot.subsystems.coral.CoralEffectorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralEffectorIOSparkMAX implements CoralEffectorIO {

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final DigitalInput effectorLimitSwitchHaveCoral = new DigitalInput(coralHaveCoralDioId);

  public CoralEffectorIOSparkMAX(int motorId) {
    motor = new SparkMax(motorId, SparkLowLevel.MotorType.kBrushless);
    encoder = motor.getEncoder();

    // effectorMotor.configure();

    SparkMaxConfig effectorConfig = new SparkMaxConfig();
    effectorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(effectorCurrentMotorLimit)
        .voltageCompensation(12.0);

    effectorConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
    effectorConfig
        .encoder
        .positionConversionFactor(effectorEncoderPositionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                effectorConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    tryUntilOk(motor, 5, () -> encoder.setPosition(0.0));
  }

  public void updateInputs(CoralEffectorIOInputs inputs) {
    inputs.speed = motor.get();
    inputs.appliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.temperature = motor.getMotorTemperature();
    inputs.haveCoral = effectorLimitSwitchHaveCoral.get();
    inputs.coralOnTheWay = false;
  }

  public void setEffectorVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }
}

// Rename Coral, add encoder, find things to add for motor.configure(); such as motorid,
// temeprature, etc.

// Check MotorIOConfig file to get important implementations to code
// Three main things to add found in Drive something
