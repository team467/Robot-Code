package frc.robot.subsystems.shooter;

import static frc.robot.Schematic.shooterBackCanId;
import static frc.robot.Schematic.shooterFrontLeftCanId;
import static frc.robot.Schematic.shooterFrontRightCanId;
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

  private final SparkMax leader;
  private final SparkMax follower;
  private final SparkMax follower2;
  private SparkClosedLoopController pidController;
  private final RelativeEncoder leaderEncoder;
  private final RelativeEncoder followerEncoder;
  private final RelativeEncoder follower2Encoder;
  private double setpointRPM = 0;

  public ShooterIOSparkMax() {
    leader = new SparkMax(shooterBackCanId, MotorType.kBrushless);
    follower = new SparkMax(shooterFrontLeftCanId, MotorType.kBrushless);
    pidController = leader.getClosedLoopController();
    follower2 = new SparkMax(shooterFrontRightCanId, MotorType.kBrushless);

    var leaderConfig = new SparkMaxConfig();
    leaderConfig
        .inverted(false)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(CURRENT_LIMIT)
        .apply(new ClosedLoopConfig().p(PID_P).i(PID_I).d(PID_D));

    var followerConfig = new SparkMaxConfig();
    followerConfig
        .inverted(false)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(CURRENT_LIMIT);

    var follower2Config = new SparkMaxConfig();
    follower2Config
        .inverted(false)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(30);

    followerConfig.follow(leader.getDeviceId(), true);
    follower2Config.follow(leader.getDeviceId(), true);

    EncoderConfig enc = new EncoderConfig();
    enc.positionConversionFactor(ENCODER_POSITION_CONVERSION);
    enc.velocityConversionFactor(ENCODER_VELOCITY_CONVERSION);
    leaderConfig.apply(enc);

    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower2.configure(
        follower2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leaderEncoder = leader.getEncoder();
    followerEncoder = follower.getEncoder();
    follower2Encoder = follower2.getEncoder();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterLeaderCurrentAmps = leader.getOutputCurrent();
    inputs.shooterLeaderAppliedVolts = leader.getBusVoltage() * leader.getAppliedOutput();
    inputs.shooterLeaderVelocityRadPerSec = leaderEncoder.getVelocity();

    inputs.shooterFollowerCurrentAmps = follower.getOutputCurrent();
    inputs.shooterFollowerAppliedVolts = follower.getBusVoltage() * follower.getAppliedOutput();
    inputs.shooterFollowerVelocityRadPerSec = followerEncoder.getVelocity();
    inputs.setpointRPM = setpointRPM;

    inputs.atSetpoint = pidController.isAtSetpoint();

    inputs.shooterFollower2CurrentAmps = follower2.getOutputCurrent();
    inputs.shooterFollower2AppliedVolts = follower2.getBusVoltage() * follower2.getAppliedOutput();
    inputs.shooterFollower2VelocityRadPerSec = follower2Encoder.getVelocity();
  }

  @Override
  public void setPercent(double percent) {
    leader.set(percent);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void stop() {
    leader.set(0);
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
    return distanceMeters * 100;
  }

  @Override
  public void setTargetDistance(double distance) {
    setTargetVelocity(distanceToRPM(distance));
  }
}
