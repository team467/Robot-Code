package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexConstants.ENCODER_INDEX_POSITION_CONVERSION;
import static frc.robot.subsystems.indexer.IndexConstants.ENCODER_INDEX_VELOCITY_CONVERSION;
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
  //  private final SparkMax feedUpMotor;
  private final DigitalInput limitSwitch;

  // private final DigitalInput limitSwitch2;

  public IndexerIOSparkMax() {
    indexerMotor = new SparkMax(INDEXER_INDEX_MOTOR_ID, MotorType.kBrushed);

    var indexerConfig = new SparkMaxConfig();
    indexerConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(30);

    EncoderConfig indexerEnc = new EncoderConfig();

    indexerEnc.positionConversionFactor(ENCODER_INDEX_POSITION_CONVERSION);

    indexerEnc.velocityConversionFactor(ENCODER_INDEX_VELOCITY_CONVERSION);

    indexerConfig.apply(indexerEnc);

    indexerMotor.configure(
        indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    limitSwitch = new DigitalInput(LIMIT_SWITCH_CHANNEL);
    // limitSwitch2 = new DigitalInput(0);

  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexPercentOutput = indexerMotor.get();
    inputs.indexVolts = indexerMotor.getBusVoltage() * indexerMotor.getAppliedOutput();
    inputs.indexAmps = indexerMotor.getOutputCurrent();
    inputs.ballAtSwitch = isSwitchPressed();
    // inputs.ballAtSwitch2 = isSwitchPressed();
  }

  @Override
  public void setPercent(double indexerPercent) {
    indexerMotor.set(indexerPercent);
  }

  @Override
  public void setVoltage(double indexerVolts) {
    indexerMotor.setVoltage(indexerVolts);
  }

  @Override
  public void stop() {
    indexerMotor.set(0);
  }

  @Override
  public boolean isSwitchPressed() {
    return limitSwitch.get();
  }
}
