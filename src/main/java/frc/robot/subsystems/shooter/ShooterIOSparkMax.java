package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;

public class ShooterIOSparkMax implements ShooterIO {

  private final SparkMax leader;
  private final SparkMax follower;
  private final SparkMax follower2;
  private final PIDController pidController = new PIDController(PID_P, PID_I, PID_D);
  private final RelativeEncoder leaderEncoder;
  private final RelativeEncoder followerEncoder;
  private final RelativeEncoder follower2Encoder;
  private double setpointRPM = 0;

  public ShooterIOSparkMax() {
    leader = new SparkMax(LEADER_MOTOR_ID, MotorType.kBrushless);
    follower = new SparkMax(FOLLOWER_MOTOR_ID, MotorType.kBrushless);
    follower2 = new SparkMax(FOLLOWER_MOTOR_ID, MotorType.kBrushless);

    var config = new SparkMaxConfig();
      config.inverted(false)
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12)
          .smartCurrentLimit(30);

      var followerConfig = new SparkMaxConfig();
      followerConfig.inverted(true)
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12)
          .smartCurrentLimit(30);

    var follower2Config = new SparkMaxConfig();
    follower2Config.inverted(true)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(30);

    followerConfig.follow(leader.getDeviceId(), true);
    follower2Config.follow(leader.getDeviceId(), true);

    EncoderConfig enc = new EncoderConfig();
    enc.positionConversionFactor(ENCODER_POSITION_CONVERSION);
    enc.velocityConversionFactor(ENCODER_VELOCITY_CONVERSION);
    config.apply(enc);

    leader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    leader.set(pidController.calculate(leaderEncoder.getVelocity(), setpointRPM));
  }
}
