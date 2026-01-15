package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIOSparkMax implements ShooterIO {

    private final SparkMax leader;
    private final SparkMax follower;
    private final RelativeEncoder leaderEncoder;
    private final RelativeEncoder followerEncoder;

    public ShooterIOSparkMax() {
        leader = new SparkMax(LEADER_MOTOR_ID, MotorType.kBrushless);
        follower = new SparkMax(FOLLOWER_MOTOR_ID, MotorType.kBrushless);

        var config = new SparkMaxConfig();
        config.inverted(false)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12)
                .smartCurrentLimit(30);
        config.follow(leader.getDeviceId());

        EncoderConfig enc = new EncoderConfig();
        enc.positionConversionFactor(ENCODER_POSITION_CONVERSION);
        enc.velocityConversionFactor(ENCODER_VELOCITY_CONVERSION);
        config.apply(enc);

        leader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leaderEncoder = leader.getEncoder();
        followerEncoder = follower.getEncoder();
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

    public void setPercent(double percent) {
        leader.set(percent);
        follower.set(percent);
    }

    public void setVoltage(double volts) {
        leader.setVoltage(volts);
    }

    public void stop() {
        leader.set(0);
        follower.set(0);
    }
}