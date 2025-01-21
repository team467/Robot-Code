package frc.robot.subsystems.coral;

import static frc.lib.utils.SparkUtil.tryUntilOk;
import static frc.robot.subsystems.coral.CoralEffectorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class CoralEffectorIOSparkMAX implements CoralEffectorIO {

  private final SparkMax effectorMotor;
  private final RelativeEncoder effectorEncoder;
  private final SparkLimitSwitch limitSwitch;

  public CoralEffectorIOSparkMAX(int motorId) {
    effectorMotor = new SparkMax(motorId, SparkLowLevel.MotorType.kBrushless);
    limitSwitch = effectorMotor.getReverseLimitSwitch();
    effectorEncoder = effectorMotor.getEncoder();
    // effectorEncoder =

    // effectorMotor.configure();

    var effectorConfig = new SparkFlexConfig();
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
        effectorMotor,
        5,
        () ->
            effectorMotor.configure(
                effectorConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    tryUntilOk(effectorMotor, 5, () -> effectorEncoder.setPosition(0.0));
  }

  private void tryUntilok(SparkMax effectorMotor, int i, Object o) {}

  public void updateInputs(EffectorIOInputs inputs) {
    inputs.appliedVolts = effectorMotor.getBusVoltage() * effectorMotor.getAppliedOutput();
    inputs.currentAmps = effectorMotor.getOutputCurrent();
    inputs.motorLimitSwitch = limitSwitch.isPressed();
  }

  public void setEffectorVoltage(double volts) {
    effectorMotor.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    effectorMotor.set(speed);
  }
}

// Rename Coral, add encoder, find things to add for motor.configure(); such as motorid,
// temeprature, etc.

// Check MotorIOConfig file to get important implementations to code
// Three main things to add found in Drive something
