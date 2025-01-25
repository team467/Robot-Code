package frc.robot.subsystems.climber;

import static frc.lib.utils.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberIOSparkMax implements ClimberIO {

  private final SparkMax climberLeader;
  private final RelativeEncoder climberLeaderEncoder;
  private final SparkMax climberFollower;
  private SparkLimitSwitch limitSwitch;

  public ClimberIOSparkMax() {
    climberLeader = new SparkMax(ClimberConstants.CLIMBER_LEADER_ID, MotorType.kBrushless);
    var ClimberLeaderConfig = new SparkMaxConfig();
    ClimberLeaderConfig.inverted(true)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(40);
    ClimberLeaderConfig.softLimit
        .forwardSoftLimit(10)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(false);
    ClimberLeaderConfig.encoder.positionConversionFactor(
        ClimberConstants.CLIMBER_CONVERSION_FACTOR);

    climberLeaderEncoder = climberLeader.getEncoder();

    climberFollower = new SparkMax(ClimberConstants.CLIMBER_FOLLOWER_ID, MotorType.kBrushless);
    var ClimberFollowerConfig = new SparkMaxConfig();
    ClimberFollowerConfig.follow(1);

    tryUntilOk(
        climberLeader,
        5,
        () ->
            climberLeader.configure(
                ClimberLeaderConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    tryUntilOk(
        climberFollower,
        5,
        () ->
            climberFollower.configure(
                ClimberFollowerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    tryUntilOk(climberLeader, 5, () -> climberLeaderEncoder.setPosition(0.0));
    limitSwitch = climberLeader.getForwardLimitSwitch();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.volts = climberLeader.getBusVoltage() * climberLeader.getAppliedOutput();
    inputs.current = climberLeader.getOutputCurrent();
    inputs.speed = climberLeader.get();
    inputs.position = climberLeaderEncoder.getPosition();
    inputs.climberWinched =
        inputs.position >= ClimberConstants.LOWER_WINCHED_POSITION
            && inputs.position <= ClimberConstants.UPPER_WINCHED_POSITION;
    inputs.climberDeployed =
        inputs.position >= ClimberConstants.LOWER_DEPLOYED_POSITION
            && inputs.position <= ClimberConstants.UPPER_DEPLOYED_POSITION;
    inputs.climberStowed = limitSwitch.isPressed();

    if (inputs.climberStowed) {
      resetPosition();
    }
  }

  @Override
  public void setVoltage(double voltage) {
    climberLeader.setVoltage(voltage);
  }

  public void setSpeed(double speed) {
    climberLeader.set(speed);
  }

  @Override
  public void resetPosition() {
    climberLeaderEncoder.setPosition(0);
  }
}
