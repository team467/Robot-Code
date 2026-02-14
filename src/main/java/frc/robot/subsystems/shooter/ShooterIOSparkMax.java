package frc.robot.subsystems.shooter;

import static frc.robot.Schematic.shooterBottomMotorCanId;
import static frc.robot.Schematic.shooterMiddleMotorCanId;
import static frc.robot.Schematic.shooterTopMotorCanId;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIOSparkMax implements ShooterIO {

  private final SparkMax middleMotor;
  private final SparkMax bottomMotor;
  private final SparkMax topMotor;
  private SparkClosedLoopController pidController;
  private final RelativeEncoder middleMotorEncoder;
  private final RelativeEncoder bottomMotorEncoder;
  private final RelativeEncoder topMotorEncoder;
  private double setpointRPM = 0;

  public ShooterIOSparkMax() {
    middleMotor = new SparkMax(shooterMiddleMotorCanId, MotorType.kBrushless);
    bottomMotor = new SparkMax(shooterBottomMotorCanId, MotorType.kBrushless);
    pidController = middleMotor.getClosedLoopController();
    topMotor = new SparkMax(shooterTopMotorCanId, MotorType.kBrushless);

    var middleMotorConfig = new SparkMaxConfig();
    middleMotorConfig
        .inverted(false)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(CURRENT_LIMIT)
        //        .closedLoopRampRate(CLOSE_RAMP_RATE)
        .apply(new ClosedLoopConfig().p(PID_P).i(PID_I).d(PID_D));

    var bottomMotorConfig = new SparkMaxConfig();
    bottomMotorConfig
        .inverted(false)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        //        .closedLoopRampRate(CLOSE_RAMP_RATE)
        .smartCurrentLimit(CURRENT_LIMIT);

    var topMotorConfig = new SparkMaxConfig();
    topMotorConfig
        .inverted(false)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        //        .closedLoopRampRate(CLOSE_RAMP_RATE)
        .smartCurrentLimit(CURRENT_LIMIT);

    bottomMotorConfig.follow(middleMotor.getDeviceId(), true);
    topMotorConfig.follow(middleMotor.getDeviceId(), true);

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
    inputs.middleMotorRPM = middleMotorEncoder.getVelocity();

    inputs.bottomMotorCurrentAmps = bottomMotor.getOutputCurrent();
    inputs.bottomMotorAppliedVolts = bottomMotor.getBusVoltage() * bottomMotor.getAppliedOutput();
    inputs.bottomMotorRPM = bottomMotorEncoder.getVelocity();

    inputs.setpointRPM = setpointRPM;
    inputs.atSetpoint = pidController.isAtSetpoint();

    inputs.topMotorCurrentAmps = topMotor.getOutputCurrent();
    inputs.topMotorAppliedVolts = topMotor.getBusVoltage() * topMotor.getAppliedOutput();
    inputs.topMotorRPM = topMotorEncoder.getVelocity();

    inputs.totalAmps = inputs.topMotorCurrentAmps + inputs.middleMotorCurrentAmps + inputs.bottomMotorCurrentAmps;
  }

  @Override
  public void setPercent(double percent) {
    middleMotor.set(percent);
  }

  @Override
  public void setVoltage(double volts) {
    middleMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    middleMotor.set(0);
  }

  @Override
  public void setTargetVelocity(double setpoint) {
    this.setpointRPM = setpoint;
  }

  @Override
  public void goToSetpoint() {
    pidController.setSetpoint(setpointRPM, ControlType.kVelocity);
  }

  private double distanceToRPM(double distanceMeters) {
    return distanceMeters * 2.79775342767 + 16.8379527141;
  }

  @Override
  public void setTargetDistance(double distance) {
    setTargetVelocity(distanceToRPM(distance));
  }
}
