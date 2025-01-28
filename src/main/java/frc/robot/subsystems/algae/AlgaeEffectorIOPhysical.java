package frc.robot.subsystems.algae;

import static frc.lib.utils.SparkUtil.tryUntilOk;
import static frc.robot.Schematic.algaePivotCanId;
import static frc.robot.Schematic.algaeRemovalCanId;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeEffectorIOPhysical implements AlgaeEffectorIO {

  // motors
  private final SparkMax pivotMotor;
  private final SparkMax removalMotor;
  private final RelativeEncoder pivotMotorEncoder;
  private final SparkLimitSwitch forwardLimitSwitch;
  private final SparkLimitSwitch reverseLimitSwitch;

  public AlgaeEffectorIOPhysical() {
    pivotMotor = new SparkMax(algaePivotCanId, MotorType.kBrushless);
    removalMotor = new SparkMax(algaeRemovalCanId, MotorType.kBrushless);

    pivotMotorEncoder = pivotMotor.getEncoder();

    forwardLimitSwitch = pivotMotor.getForwardLimitSwitch();
    reverseLimitSwitch = pivotMotor.getReverseLimitSwitch();

    SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    pivotMotorConfig.idleMode(IdleMode.kBrake);
    pivotMotorConfig.inverted(AlgaeEffectorConstants.PIVOT_INVERTED);
    pivotMotorConfig.voltageCompensation(12);

    EncoderConfig pivotEncoderConfig = new EncoderConfig();
    pivotEncoderConfig.positionConversionFactor(
        360
            / AlgaeEffectorConstants.NEO_PULSES_PER_REVOLUTION
            * AlgaeEffectorConstants.PIVOT_GEAR_RATIO);
    pivotMotorConfig.apply(pivotEncoderConfig);

    tryUntilOk(
        pivotMotor,
        5,
        () ->
            pivotMotor.configure(
                pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig removalMotorConfig = new SparkMaxConfig();
    removalMotorConfig.idleMode(IdleMode.kBrake);
    removalMotorConfig.inverted(AlgaeEffectorConstants.REMOVAL_INVERTED);
    removalMotorConfig.voltageCompensation(12);

    tryUntilOk(
        removalMotor,
        5,
        () ->
            removalMotor.configure(
                removalMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  public void setRemovalVolts(double volts) {
    removalMotor.setVoltage(volts);
  }

  public void setPivotVolts(double volts) {
    pivotMotor.setVoltage(volts);
  }

  public void updateInputs(AlgaeEffectorIOInputs inputs) {
    inputs.removalVolts = removalMotor.getBusVoltage() * removalMotor.getAppliedOutput();
    inputs.pivotVolts = pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput();
    inputs.pivotVelocity = pivotMotorEncoder.getVelocity();
    inputs.removalAmps = removalMotor.getOutputCurrent();
    inputs.pivotAmps = pivotMotor.getOutputCurrent();
    inputs.pivotPosition = pivotMotorEncoder.getPosition();
    inputs.isFullyExtended = forwardLimitSwitch.isPressed();
    inputs.isStowed = reverseLimitSwitch.isPressed();
    inputs.pivotMotorTemp = pivotMotor.getMotorTemperature();
    inputs.removalMotorTemp = removalMotor.getMotorTemperature();
  }
}
