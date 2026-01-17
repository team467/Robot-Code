package frc.robot.subsystems.climber;

import static frc.lib.utils.SparkUtil.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIOPhysical implements ClimberIO {
  private final SparkMax spark;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final DigitalInput limitSwitch;

  private boolean isCalibrated = false;
  private double targetRotation;

  public ClimberIOPhysical() {
    spark = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless);
    encoder = spark.getEncoder();
    controller = spark.getClosedLoopController();
    limitSwitch = new DigitalInput(LIMIT_SWITCH_ID);

    var config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(CLIMBER_CURRENT_LIMIT)
        .voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(ENCODER_CONVERSION_FACTOR)
        .velocityConversionFactor(ENCODER_CONVERSION_FACTOR / 60);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(false)
        .pidf(CLIMBER_KP, CLIMBER_KI, CLIMBER_KD, 0.0);
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
    tryUntilOk(spark, 5, () -> encoder.setPosition(STARTING_DEGREES));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.positionDegrees = encoder.getPosition();
    inputs.velocityDegreesPerSec = encoder.getVelocity();
    inputs.appliedVolts = spark.getBusVoltage() * spark.getAppliedOutput();
    inputs.currentAmps = spark.getOutputCurrent();
    inputs.targetRotation = targetRotation;
    inputs.atTargetRotation =
        isCalibrated && Math.abs(targetRotation - inputs.positionDegrees) < TOLERANCE;
    inputs.limitSwitch = !limitSwitch.get();

    if (inputs.limitSwitch) {
      this.isCalibrated = true;
      encoder.setPosition(CALIBRATION_POSITION_DEGREES);
    }
    inputs.isCalibrated = this.isCalibrated;
  }

  @Override
  public void setPercent(double percent) {
    spark.set(percent);
  }

  @Override
  public void setRotation(double degrees) {
    this.targetRotation = degrees;
  }

  @Override
  public void goToRotation() {
    if (!isCalibrated) {
      setPercent(CALIBRATION_PERCENT);
    } else {
      controller.setReference(this.targetRotation, SparkBase.ControlType.kPosition);
    }
  }
}
