package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexConstants.ENCODER_POSITION_CONVERSION;
import static frc.robot.subsystems.indexer.IndexConstants.ENCODER_VELOCITY_CONVERSION;
import static frc.robot.subsystems.indexer.IndexConstants.INDEXER_FEEDUP_ID;
import static frc.robot.subsystems.indexer.IndexConstants.INDEXER_INDEXERMOTOR_ID;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IndexerIOSparkMax implements IndexerIO {

  private final SparkMax indexerMotor;
  private final SparkMax feedUpMotor;
  // private final DigitalInput limitSwitch;
  // private final DigitalInput limitSwitch2;

  public IndexerIOSparkMax() {
    indexerMotor = new SparkMax(INDEXER_INDEXERMOTOR_ID, MotorType.kBrushed);
    feedUpMotor = new SparkMax(INDEXER_FEEDUP_ID, MotorType.kBrushed);

    var config = new SparkMaxConfig();
    config.inverted(true).idleMode(IdleMode.kBrake).voltageCompensation(12).smartCurrentLimit(30);

    EncoderConfig enc = new EncoderConfig();
    enc.positionConversionFactor(ENCODER_POSITION_CONVERSION);
    enc.velocityConversionFactor(ENCODER_VELOCITY_CONVERSION);
    config.apply(enc);

    indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // limitSwitch = new DigitalInput(0);
    // limitSwitch2 = new DigitalInput(0);

  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.percentOutput = indexerMotor.get();
    inputs.percentOutput = feedUpMotor.get();
    inputs.volts = indexerMotor.getBusVoltage() * indexerMotor.getAppliedOutput();
    inputs.volts = feedUpMotor.getBusVoltage() * feedUpMotor.getAppliedOutput();
    inputs.amps = indexerMotor.getOutputCurrent();
    inputs.amps = feedUpMotor.getOutputCurrent();
    // inputs.ballAtSwitch = isSwitchPressed();
    // inputs.ballAtSwitch2 = isSwitchPressed();
  }

  @Override
  public void setPercent(double percent) {
    indexerMotor.set(percent);
    feedUpMotor.set(percent);
  }

  public void setVoltage(double volts) {
    indexerMotor.setVoltage(volts);
    feedUpMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    indexerMotor.set(0);
    feedUpMotor.set(0);
  }

  /*public boolean isSwitchPressed() {
    return limitSwitch.get();
  }*/
}
