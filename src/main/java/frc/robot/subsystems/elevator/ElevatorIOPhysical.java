package frc.robot.subsystems.elevator;

import static frc.lib.utils.SparkUtil.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.Schematic;

public class ElevatorIOPhysical implements ElevatorIO {
  private final SparkMax spark;
  private final RelativeEncoder encoder;
  private final SparkLimitSwitch elevatorStowLimitSwitch;

  private final SparkClosedLoopController controller;
  private double setpoint;
  public ElevatorIOPhysical() {
    spark = new SparkMax(Schematic.elevatorMotorID, MotorType.kBrushless);
    encoder = spark.getEncoder();
    elevatorStowLimitSwitch = spark.getReverseLimitSwitch();

    controller = spark.getClosedLoopController();

    var config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(elevatorCurrentLimit)
        .voltageCompensation(12.0)
        .softLimit
        .forwardSoftLimit(Units.inchesToMeters(73))
        .forwardSoftLimitEnabled(true);
    config
        .encoder
        .positionConversionFactor(ENCODER_CONVERSION_FACTOR)
        .velocityConversionFactor(ENCODER_CONVERSION_FACTOR / 60)
        .uvwAverageDepth(10)
        .uvwAverageDepth(2);
    config
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(false)
        .pidf(4.6, 0.0, 24.0, 0.0);
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(spark, 5, () -> encoder.setPosition(0.0));
  }

  public void updateInputs(ElevatorIOInputs inputs) {

    inputs.positionMeters = encoder.getPosition();
    inputs.velocityMetersPerSec = encoder.getVelocity();
    inputs.elevatorAppliedVolts = spark.getBusVoltage() * spark.getAppliedOutput();
    inputs.elevatorCurrentAmps = spark.getOutputCurrent();
    inputs.limitSwitchPressed = elevatorStowLimitSwitch.isPressed();
    inputs.elevatorSetpoint = setpoint;
    inputs.atSetpoint = Math.abs(setpoint - inputs.positionMeters) < TOLERANCE;
  }

  @Override
  public void setPercent(double percent) {
    spark.set(percent);
  }

  @Override
  public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void setPosition(double position) {
    controller.setReference(position, SparkBase.ControlType.kPosition);
    setpoint = position;
  }

  @Override
  public void resetPosition(double positionMeters) {
    encoder.setPosition(positionMeters);
  }
}
