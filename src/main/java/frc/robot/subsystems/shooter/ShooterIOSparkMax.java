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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.proto.SimpleMotorFeedforwardProto;

public class ShooterIOSparkMax implements ShooterIO {

  private final SparkMax middleMotor;
  private final SparkMax bottomMotor;
  private final SparkMax topMotor;
  private final RelativeEncoder middleMotorEncoder;
  private final RelativeEncoder bottomMotorEncoder;
  private final RelativeEncoder topMotorEncoder;
  private final SimpleMotorFeedforward middleMotorFeedForward;
  private final SimpleMotorFeedforward topMotorFeedForward;
  private final SimpleMotorFeedforward bottomMotorFeedForward;

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
        .inverted(true)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(CURRENT_LIMIT);

    var topMotorConfig = new SparkMaxConfig();
    topMotorConfig
        .inverted(true)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(CURRENT_LIMIT);

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

    middleMotorFeedForward = new SimpleMotorFeedforward(KS, KV, KA);
    topMotorFeedForward = new SimpleMotorFeedforward(KS, KV, KA);
    bottomMotorFeedForward = new SimpleMotorFeedforward(KS, KV, KA);
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
  public void goToSpeed(double volts, double rpm) {
    middleMotor.setVoltage(volts + middleMotorFeedForward.calculateWithVelocities(middleMotorEncoder.getVelocity(), rpm));
    topMotor.setVoltage(volts + topMotorFeedForward.calculateWithVelocities(topMotorEncoder.getVelocity(), rpm));
    bottomMotor.setVoltage(volts + bottomMotorFeedForward.calculateWithVelocities(bottomMotorEncoder.getVelocity(), rpm));
  }

  @Override
  public void stop() {
    middleMotor.set(0);
  }

  private double distanceToRPM(double distanceMeters) {
    return distanceMeters * 2.79775342767 + 16.8379527141;
  }
}
