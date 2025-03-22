package frc.robot.subsystems.elevator;

import static frc.lib.utils.SparkUtil.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.Schematic;

public class ElevatorIOPhysical implements ElevatorIO {
  private final SparkMax spark;
  private final RelativeEncoder encoder;
  private final DigitalInput elevatorStowLimitSwitch;

  private boolean zeroedOnce = false;

  private final SparkClosedLoopController controller;
  private final SparkClosedLoopController holdController;
  private double setpoint;

  public ElevatorIOPhysical() {
    spark = new SparkMax(Schematic.elevatorMotorID, MotorType.kBrushless);
    encoder = spark.getAlternateEncoder();
    elevatorStowLimitSwitch = new DigitalInput(3);

    controller = spark.getClosedLoopController();
    holdController = spark.getClosedLoopController();

    var config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(elevatorCurrentLimit)
        .voltageCompensation(12.0);
    config
        .alternateEncoder
        .positionConversionFactor(ENCODER_CONVERSION_FACTOR)
        .velocityConversionFactor(ENCODER_CONVERSION_FACTOR / 60)
        .inverted(true)
        .averageDepth(2);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .positionWrappingEnabled(false)
        .pidf(8.0, 0.0, 5, 0.0); // p:4.6 d: 24
    config
        .signals
        .externalOrAltEncoderPositionAlwaysOn(true)
        .externalOrAltEncoderPosition(20)
        .externalOrAltEncoderVelocityAlwaysOn(true)
        .externalOrAltEncoderVelocity(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    config
        .softLimit
        .forwardSoftLimit(0.79)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(false);
    var holdConfig = new SparkMaxConfig();
    holdConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(elevatorCurrentLimit)
        .voltageCompensation(12.0);
    holdConfig
        .alternateEncoder
        .positionConversionFactor(ENCODER_CONVERSION_FACTOR)
        .velocityConversionFactor(ENCODER_CONVERSION_FACTOR / 60)
        .inverted(true)
        .averageDepth(2);
    holdConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .positionWrappingEnabled(false)
        .pidf(0.5, 0.0, 12.0, 0.0); // Different PIDF values for the hold controller
    holdConfig
        .signals
        .externalOrAltEncoderPositionAlwaysOn(true)
        .externalOrAltEncoderPosition(20)
        .externalOrAltEncoderVelocityAlwaysOn(true)
        .externalOrAltEncoderVelocity(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    holdConfig
        .softLimit
        .forwardSoftLimit(0.79)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(false);

    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(spark, 5, () -> encoder.setPosition(0.0));
    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                holdConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(spark, 5, () -> encoder.setPosition(0.0));
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.positionMeters = encoder.getPosition();
    inputs.velocityMetersPerSec = encoder.getVelocity();
    inputs.elevatorAppliedVolts = spark.getBusVoltage() * spark.getAppliedOutput();
    spark.getAppliedOutput();
    inputs.elevatorCurrentAmps = spark.getOutputCurrent();
    inputs.elevatorSetpoint = setpoint;
    inputs.atSetpoint = Math.abs(setpoint - inputs.positionMeters) < TOLERANCE;
    inputs.stowLimitSwitch = !elevatorStowLimitSwitch.get();
    inputs.isCalibrated = zeroedOnce;
  }

  @Override
  public void setPercent(double percent) {
    if (percent > 0.7) {
      spark.set(0.7);
    } else if (percent < -0.7) {
      spark.set(-0.7);
    } else {
      spark.set(percent);
    }
  }

  @Override
  public void setVoltage(double volts) {
    if (!zeroedOnce && volts < 0) {
      spark.setVoltage(-2.0);
    } else if (volts > 8.0) {
      spark.setVoltage(8.0);
    } else if (volts < -6.0) {
      spark.setVoltage(-6.0);
    } else if (encoder.getPosition() < ReefHeight.L2.height && volts < -2.0) {
      spark.setVoltage(-2.0);
    } else {
      spark.setVoltage(volts);
    }
  }
  // 0.9
  // 1.44

  @Override
  public void setPosition(double position) {
    controller.setReference(position, SparkBase.ControlType.kPosition);
    setpoint = position;
  }

  @Override
  public void resetPosition(double positionMeters) {
    zeroedOnce = true;
    encoder.setPosition(positionMeters);
  }

  @Override
  public void hold(double holdPosition) {
    holdController.setReference(holdPosition, SparkBase.ControlType.kPosition);
  }

  @Override
  public void goToSetpoint(double setpoint) {
    controller.setReference(setpoint, SparkBase.ControlType.kPosition);
  }
}
