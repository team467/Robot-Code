package frc.robot.subsystems.shooter;

import static frc.robot.Schematic.shooterBottomMotorCanId;
import static frc.robot.Schematic.shooterMiddleMotorCanId;
import static frc.robot.Schematic.shooterTopMotorCanId;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIOSparkMax implements ShooterIO {

  private final SparkMax middleMotor;
  private final SparkMax bottomMotor;
  private final SparkMax topMotor;
  private final RelativeEncoder middleMotorEncoder;
  private final RelativeEncoder bottomMotorEncoder;
  private final RelativeEncoder topMotorEncoder;

  public ShooterIOSparkMax() {
    middleMotor = new SparkMax(shooterMiddleMotorCanId, MotorType.kBrushless);
    bottomMotor = new SparkMax(shooterBottomMotorCanId, MotorType.kBrushless);
    topMotor = new SparkMax(shooterTopMotorCanId, MotorType.kBrushless);

    var middleMotorConfig = new SparkMaxConfig();
    middleMotorConfig
        .inverted(false)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(CURRENT_LIMIT);

    var bottomMotorConfig = new SparkMaxConfig();
    bottomMotorConfig
        .inverted(false)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(CURRENT_LIMIT);

    var topMotorConfig = new SparkMaxConfig();
    topMotorConfig
        .inverted(false)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(CURRENT_LIMIT);

    topMotorConfig.follow(middleMotor.getDeviceId(), true);
    bottomMotorConfig.follow(middleMotor.getDeviceId(), true);

    EncoderConfig enc = new EncoderConfig();
    enc.positionConversionFactor(ENCODER_POSITION_CONVERSION);
    enc.velocityConversionFactor(ENCODER_VELOCITY_CONVERSION);
    middleMotorConfig.apply(enc);

    middleMotor.configure(
        middleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomMotor.configure(
        bottomMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topMotor.configure(
        topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    middleMotorEncoder = middleMotor.getEncoder();
    bottomMotorEncoder = bottomMotor.getEncoder();
    topMotorEncoder = topMotor.getEncoder();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.middleMotorCurrentAmps = middleMotor.getOutputCurrent();
    inputs.middleMotorAppliedVolts = middleMotor.getBusVoltage() * middleMotor.getAppliedOutput();
    inputs.middleMotorVelocityRadPerSec = middleMotorEncoder.getVelocity();

    inputs.bottomMotorCurrentAmps = bottomMotor.getOutputCurrent();
    inputs.bottomMotorAppliedVolts = bottomMotor.getBusVoltage() * bottomMotor.getAppliedOutput();
    inputs.bottomMotorVelocityRadPerSec = bottomMotorEncoder.getVelocity();

    inputs.topMotorCurrentAmps = topMotor.getOutputCurrent();
    inputs.topMotorAppliedVolts = topMotor.getBusVoltage() * topMotor.getAppliedOutput();
    inputs.topMotorVelocityRadPerSec = topMotorEncoder.getVelocity();

    inputs.totalAmps =
        inputs.middleMotorCurrentAmps + inputs.bottomMotorCurrentAmps + inputs.topMotorCurrentAmps;

    inputs.shooterWheelVelocityRadPerSec =
        inputs.middleMotorVelocityRadPerSec / SHOOTER_WHEEL_GEAR_RATIO;
    inputs.shooterWheelPosition = middleMotorEncoder.getPosition() / SHOOTER_WHEEL_GEAR_RATIO;
  }

  @Override
  public void setVoltage(double volts) {
    middleMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    middleMotor.set(0);
  }
}
