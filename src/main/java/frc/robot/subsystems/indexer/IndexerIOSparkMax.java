package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexConstants.ENCODER_FEEDUP_POSITION_CONVERSION;
import static frc.robot.subsystems.indexer.IndexConstants.ENCODER_FEEDUP_VELOCITY_CONVERSION;
import static frc.robot.subsystems.indexer.IndexConstants.ENCODER_INDEX_POSITION_CONVERSION;
import static frc.robot.subsystems.indexer.IndexConstants.ENCODER_INDEX_VELOCITY_CONVERSION;
import static frc.robot.subsystems.indexer.IndexConstants.INDEXER_FEEDUP_ID;
import static frc.robot.subsystems.indexer.IndexConstants.INDEXER_INDEX_MOTOR_ID;
import static frc.robot.subsystems.indexer.IndexConstants.LIMIT_SWITCH_CHANNEL;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOSparkMax implements IndexerIO {

  private final SparkMax indexerMotor;
  private final SparkMax feedUpMotor;
  private final DigitalInput limitSwitch;

  // private final DigitalInput limitSwitch2;

  public IndexerIOSparkMax() {
    indexerMotor = new SparkMax(INDEXER_INDEX_MOTOR_ID, MotorType.kBrushed);
    feedUpMotor = new SparkMax(INDEXER_FEEDUP_ID, MotorType.kBrushed);

    var indexerConfig = new SparkMaxConfig();
    var feedUpConfig = new SparkMaxConfig();
    indexerConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(30);
    feedUpConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(30);

    EncoderConfig indexerEnc = new EncoderConfig();
    EncoderConfig feederUpEnc = new EncoderConfig();

    indexerEnc.positionConversionFactor(ENCODER_INDEX_POSITION_CONVERSION);
    feederUpEnc.positionConversionFactor(ENCODER_FEEDUP_POSITION_CONVERSION);

    indexerEnc.velocityConversionFactor(ENCODER_INDEX_VELOCITY_CONVERSION);
    feederUpEnc.velocityConversionFactor(ENCODER_FEEDUP_VELOCITY_CONVERSION);

    indexerConfig.apply(indexerEnc);
    feedUpConfig.apply(feederUpEnc);

    indexerMotor.configure(
        indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feedUpMotor.configure(
        feedUpConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    limitSwitch = new DigitalInput(LIMIT_SWITCH_CHANNEL);
    // limitSwitch2 = new DigitalInput(0);

  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexPercentOutput = indexerMotor.get();
    inputs.feedUpPercentOutput = feedUpMotor.get();
    inputs.indexVolts = indexerMotor.getBusVoltage() * indexerMotor.getAppliedOutput();
    inputs.feedUpVolts = feedUpMotor.getBusVoltage() * feedUpMotor.getAppliedOutput();
    inputs.indexAmps = indexerMotor.getOutputCurrent();
    inputs.feedUpAmps = feedUpMotor.getOutputCurrent();
    inputs.ballAtSwitch = isSwitchPressed();
    // inputs.ballAtSwitch2 = isSwitchPressed();
  }

  @Override
  public void setPercent(double indexerPercent, double feedUpPercent) {
    indexerMotor.set(indexerPercent);
    feedUpMotor.set(feedUpPercent);
  }

  @Override
  public void setVoltage(double indexerVolts, double feedUpVolts) {
    indexerMotor.setVoltage(indexerVolts);
    feedUpMotor.setVoltage(feedUpVolts);
  }

  @Override
  public void stop() {
    indexerMotor.set(0);
    feedUpMotor.set(0);
  }

  @Override
  public boolean isSwitchPressed() {
    return limitSwitch.get();
  }
}
