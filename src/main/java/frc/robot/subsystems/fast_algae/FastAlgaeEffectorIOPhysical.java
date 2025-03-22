package frc.robot.subsystems.fast_algae;

import static frc.lib.utils.SparkUtil.tryUntilOk;
import static frc.robot.Schematic.algaePivotCanId;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class FastAlgaeEffectorIOPhysical implements FastAlgaeEffectorIO {

  // motors
  private final SparkMax pivotMotor;
  private final RelativeEncoder pivotMotorEncoder;

  public FastAlgaeEffectorIOPhysical() {
    pivotMotor = new SparkMax(algaePivotCanId, MotorType.kBrushless);

    pivotMotorEncoder = pivotMotor.getEncoder();

    SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    pivotMotorConfig.idleMode(IdleMode.kBrake);
    // pivotMotorConfig.inverted(FastAlgaeEffectorConstants.PIVOT_INVERTED);
    pivotMotorConfig.voltageCompensation(12);

    EncoderConfig pivotEncoderConfig = new EncoderConfig();
    pivotEncoderConfig.positionConversionFactor(
        360
            / FastAlgaeEffectorConstants.NEO_PULSES_PER_REVOLUTION
            * FastAlgaeEffectorConstants.PIVOT_GEAR_RATIO);
    pivotMotorConfig.apply(pivotEncoderConfig);

    pivotMotorConfig.smartCurrentLimit(FastAlgaeEffectorConstants.PIVOT_MOTOR_CURRENT_LIMIT);

    tryUntilOk(
        pivotMotor,
        5,
        () ->
            pivotMotor.configure(
                pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void setPivotVolts(double volts) {
    pivotMotor.setVoltage(volts);
  }

  public void resetPivotPosition() {
    pivotMotorEncoder.setPosition(FastAlgaeEffectorConstants.ZERO);
  }

  public void updateInputs(FastAlgaeEffectorIOInputs inputs) {
    inputs.pivotVolts = pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput();
    inputs.pivotVelocity = pivotMotorEncoder.getVelocity();
    inputs.pivotAmps = pivotMotor.getOutputCurrent();
    inputs.pivotPosition = pivotMotorEncoder.getPosition();
    inputs.isHighPostion =
        (pivotMotorEncoder.getPosition() > FastAlgaeEffectorConstants.HIGH_ANGLE);
    inputs.isLowPostion = (pivotMotorEncoder.getPosition() > FastAlgaeEffectorConstants.LOW_ANGLE);
    inputs.isStowed =
        (inputs.pivotVolts < 0)
            && Math.abs(inputs.pivotVelocity) <= 0.1
            && (FastAlgaeEffector.stowTimer.get() > 0.2);
    inputs.pivotMotorTemp = pivotMotor.getMotorTemperature();
  }
}
