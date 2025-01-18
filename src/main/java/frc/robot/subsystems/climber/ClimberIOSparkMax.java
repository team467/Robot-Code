package frc.robot.subsystems.climber;

import static frc.lib.utils.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ClimberIOSparkMax implements ClimberIO {

  private final SparkFlex climberLeader;
  private final RelativeEncoder climberLeaderEncoder;
  private final SparkFlex climberFollower;
  private SparkLimitSwitch limitSwitch;
  private final int CLIMBER_LEADER_ID = 1;
  private final int CLIMBER_FOLLOWER_ID = 2;

  /**
   * Constructor initializes the climber system, including motors, encoders, limit switches, and
   * ratchet.
   */
  public ClimberIOSparkMax() {
    climberLeader = new SparkFlex(CLIMBER_LEADER_ID, MotorType.kBrushless);
    var ClimberLeaderConfig = new SparkFlexConfig(); // Sets configuration for the leader motor
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

    climberFollower = new SparkFlex(CLIMBER_FOLLOWER_ID, MotorType.kBrushless);
    var ClimberFollowerConfig = new SparkFlexConfig();
    ClimberFollowerConfig.follow(1); // Set the follower motor to mirror the leader motor

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
    inputs.climberAtTop = inputs.position >= ClimberConstants.WINCHED_POSITION;
    inputs.climberDeployed = inputs.position >= ClimberConstants.DEPLOYED_POSITION;
    inputs.climberStowed = limitSwitch.isPressed();

    // Reset position if the stowed limit switch is pressed
    if (inputs.climberStowed) {
      resetPosition();
    }
  }

  @Override
  // Sets the motorss percent output
  public void setVoltage(double voltage) {
    climberLeader.setVoltage(climberLeader.getAppliedOutput());
  }

  public void setSpeed(double speed) {
    climberLeader.set(speed);
  }

  @Override
  public void resetPosition() {
    climberLeaderEncoder.setPosition(0);
  }
}
