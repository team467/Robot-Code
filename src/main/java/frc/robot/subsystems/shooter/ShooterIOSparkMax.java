package frc.robot.subsystems.shooter;

import static frc.robot.Schematic.shooterBottomLeftMotorCanId;
import static frc.robot.Schematic.shooterBottomRightMotorCanId;
import static frc.robot.Schematic.shooterTopLeftMotorCanId;
import static frc.robot.Schematic.shooterTopRightMotorCanId;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIOSparkMax implements ShooterIO {

  private final SparkMax topLeftMotor;
  private final SparkMax topRightMotor;
  private final SparkMax bottomLeftMotor;
  private final SparkMax bottomRightMotor;
  private final RelativeEncoder topLeftMotorEncoder;
  private final RelativeEncoder topRightMotorEncoder;
  private final RelativeEncoder bottomLeftMotorEncoder;
  private final RelativeEncoder bottomRightMotorEncoder;
  private final boolean independentMode;

  public ShooterIOSparkMax() {
    this(false);
  }

  public ShooterIOSparkMax(boolean independentMode) {
    this.independentMode = independentMode;
    topLeftMotor = new SparkMax(shooterTopLeftMotorCanId, MotorType.kBrushless);
    topRightMotor = new SparkMax(shooterTopRightMotorCanId, MotorType.kBrushless);
    bottomLeftMotor = new SparkMax(shooterBottomLeftMotorCanId, MotorType.kBrushless);
    bottomRightMotor = new SparkMax(shooterBottomRightMotorCanId, MotorType.kBrushless);

    EncoderConfig enc = new EncoderConfig();
    enc.positionConversionFactor(ENCODER_POSITION_CONVERSION);
    enc.velocityConversionFactor(ENCODER_VELOCITY_CONVERSION);

    var bottomLeftMotorConfig = new SparkMaxConfig();
    bottomLeftMotorConfig
        .inverted(false)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(CURRENT_LIMIT);
    bottomLeftMotorConfig.apply(enc);

    var topLeftMotorConfig = new SparkMaxConfig();
    topLeftMotorConfig
        .inverted(true)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(CURRENT_LIMIT);
    topLeftMotorConfig.follow(bottomLeftMotor.getDeviceId(), false);
    topLeftMotorConfig.apply(enc);

    var bottomRightMotorConfig = new SparkMaxConfig();
    bottomRightMotorConfig
        .inverted(true)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(CURRENT_LIMIT);
    bottomRightMotorConfig.follow(bottomLeftMotor.getDeviceId(), true);
    bottomRightMotorConfig.apply(enc);

    var topRightMotorConfig = new SparkMaxConfig();
    topRightMotorConfig
        .inverted(false)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(CURRENT_LIMIT);
    topRightMotorConfig.follow(bottomLeftMotor.getDeviceId(), true);
    topRightMotorConfig.apply(enc);

    bottomLeftMotor.configure(
        bottomLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topLeftMotor.configure(
        topLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomRightMotor.configure(
        bottomRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topRightMotor.configure(
        topRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    topLeftMotorEncoder = topLeftMotor.getEncoder();
    topRightMotorEncoder = topRightMotor.getEncoder();
    bottomLeftMotorEncoder = bottomLeftMotor.getEncoder();
    bottomRightMotorEncoder = bottomRightMotor.getEncoder();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.topLeftMotorCurrentAmps = topLeftMotor.getOutputCurrent();
    inputs.topLeftMotorAppliedVolts =
        topLeftMotor.getBusVoltage() * topLeftMotor.getAppliedOutput();
    inputs.topLeftMotorVelocityRadPerSec = topLeftMotorEncoder.getVelocity();

    inputs.topRightMotorCurrentAmps = topRightMotor.getOutputCurrent();
    inputs.topRightMotorAppliedVolts =
        topRightMotor.getBusVoltage() * topRightMotor.getAppliedOutput();
    inputs.topRightMotorVelocityRadPerSec = topRightMotorEncoder.getVelocity();

    inputs.bottomLeftMotorCurrentAmps = bottomLeftMotor.getOutputCurrent();
    inputs.bottomLeftMotorAppliedVolts =
        bottomLeftMotor.getBusVoltage() * bottomLeftMotor.getAppliedOutput();
    inputs.bottomLeftMotorVelocityRadPerSec = bottomLeftMotorEncoder.getVelocity();

    inputs.bottomRightMotorCurrentAmps = bottomRightMotor.getOutputCurrent();
    inputs.bottomRightMotorAppliedVolts =
        bottomRightMotor.getBusVoltage() * bottomRightMotor.getAppliedOutput();
    inputs.bottomRightMotorVelocityRadPerSec = bottomRightMotorEncoder.getVelocity();

    inputs.totalAmps =
        inputs.topLeftMotorCurrentAmps
            + inputs.topRightMotorCurrentAmps
            + inputs.bottomLeftMotorCurrentAmps
            + inputs.bottomRightMotorCurrentAmps;

    inputs.shooterWheelVelocityRadPerSec =
        inputs.bottomLeftMotorVelocityRadPerSec / SHOOTER_WHEEL_GEAR_RATIO;
    inputs.shooterWheelPosition = bottomLeftMotorEncoder.getPosition() / SHOOTER_WHEEL_GEAR_RATIO;
  }

  @Override
  public void setVoltage(double volts) {
    bottomLeftMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    topLeftMotor.set(0);
    topRightMotor.set(0);
    bottomLeftMotor.set(0);
    bottomRightMotor.set(0);
  }

  private double distanceToRPM(double distanceMeters) {
    return distanceMeters * 2.79775342767 + 16.8379527141;
  }
}
