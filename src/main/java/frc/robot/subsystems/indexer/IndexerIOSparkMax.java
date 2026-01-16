package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOSparkMax implements IndexerIO {

  private final SparkMax leader;
  private final SparkMax follower;
  private final DigitalInput limitSwitch;
  private final DigitalInput limitSwitch2;

  public IndexerIOSparkMax() {
    leader = new SparkMax(INDEXER_LEADER_ID, MotorType.kBrushed);
    follower = new SparkMax(INDEXER_FOLLOWER_ID, MotorType.kBrushed);

    var config = new SparkMaxConfig();
    config.inverted(true)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(30);

    EncoderConfig enc = new EncoderConfig();
    enc.positionConversionFactor(ENCODER_POSITION_CONVERSION);
    enc.velocityConversionFactor(ENCODER_VELOCITY_CONVERSION);
    config.apply(enc);


    leader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    limitSwitch = new DigitalInput(0);
    limitSwitch2 = new DigitalInput(0);

  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.percentOutput = leader.get();
    inputs.percentOutput = follower.get();
    inputs.volts = leader.getBusVoltage() * leader.getAppliedOutput();
    inputs.volts = follower.getBusVoltage() * follower.getAppliedOutput();
    inputs.amps = leader.getOutputCurrent();
    inputs.amps = follower.getOutputCurrent();
    inputs.ballAtSwitch = isSwitchPressed();
    inputs.ballAtSwitch2 = isSwitchPressed();
  }

  @Override
  public void setPercent(double percent) {
    leader.set(percent);
    //follower.set(percent);
  }

  public void setVoltage(double volts) {
    leader.setVoltage(volts);
    //follower.setVoltage(volts);
  }
  @Override
  public void stop() {
    leader.set(0);
    //follower.set(0);
  }

  public boolean isSwitchPressed() {
    return limitSwitch.get();
  }
}
