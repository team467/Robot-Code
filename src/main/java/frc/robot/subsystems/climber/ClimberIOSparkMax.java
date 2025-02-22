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
  private final SparkLimitSwitch limitSwitch;

  /**
   * Constructor initializes the climber system, including motors, encoders, limit switches, and
   * ratchet.
   */
  public ClimberIOSparkMax() {
    climberLeader = new SparkMax(ClimberConstants.CLIMBER_LEADER_ID, MotorType.kBrushless);
    var ClimberLeaderConfig = new SparkMaxConfig();
    ClimberLeaderConfig.inverted(false)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(40);
    ClimberLeaderConfig.encoder.positionConversionFactor(
        ClimberConstants.CLIMBER_CONVERSION_FACTOR);

    climberLeaderEncoder = climberLeader.getEncoder();

    climberFollower = new SparkMax(ClimberConstants.CLIMBER_FOLLOWER_ID, MotorType.kBrushless);
    var ClimberFollowerConfig = new SparkMaxConfig();
    ClimberFollowerConfig.follow(ClimberConstants.CLIMBER_LEADER_ID, true);

    // Configure the leader motor using the configuration object and retry up to 5 times if it fails
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

    // Set the encoder position to 0 retrying up to 5 times if it fails
    tryUntilOk(climberLeader, 5, () -> climberLeaderEncoder.setPosition(0.0));
    limitSwitch = climberLeader.getForwardLimitSwitch();
  }

  @Override
  // Updates all the inputs
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.volts = climberLeader.getBusVoltage() * climberLeader.getAppliedOutput();
    inputs.current = climberLeader.getOutputCurrent();
    inputs.speed = climberLeader.get();
    inputs.position = climberLeaderEncoder.getPosition();
    inputs.climberWinched =
        inputs.position <= ClimberConstants.LOWER_WINCHED_POSITION
            && inputs.position >= ClimberConstants.UPPER_WINCHED_POSITION;
    inputs.climberDeployed = limitSwitch.isPressed();

    // Reset position if the stowed limit switch is pressed
    if (inputs.climberDeployed) {
      resetPosition();
    }
  }

  @Override
  // Sets the motors voltage
  public void setVoltage(double voltage) {
    climberLeader.setVoltage(voltage);
  }

  // Sets the motors speed
  public void setSpeed(double speed) {
    climberLeader.set(speed);
  }

  @Override
  public void resetPosition() {
    climberLeaderEncoder.setPosition(0);
  }

  @Override
  public void hold() {
    if (climberLeaderEncoder.getPosition() > ClimberConstants.WINCHED_POSITION) {
      climberLeader.setVoltage(-0.15);
    } else if (climberLeaderEncoder.getPosition() < ClimberConstants.WINCHED_POSITION) {
      climberLeader.setVoltage(0.15);
    }
  }
}
