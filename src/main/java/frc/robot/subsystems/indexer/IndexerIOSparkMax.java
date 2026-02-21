package frc.robot.subsystems.indexer;

import static frc.robot.Schematic.indexerFeedupCanId;
import static frc.robot.Schematic.indexerLeftLimitSwitchDIO;
import static frc.robot.Schematic.indexerRightLimitSwitchDIO;
import static frc.robot.subsystems.indexer.IndexConstants.ENCODER_FEEDUP_POSITION_CONVERSION;
import static frc.robot.subsystems.indexer.IndexConstants.ENCODER_FEEDUP_VELOCITY_CONVERSION;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOSparkMax implements IndexerIO {

  private final SparkMax feedUpMotor;

  private final DigitalInput leftLimitSwitch;
  private final DigitalInput rightLimitSwitch;

  public IndexerIOSparkMax() {
    feedUpMotor = new SparkMax(indexerFeedupCanId, MotorType.kBrushless);

    var feedUpConfig = new SparkMaxConfig();
    feedUpConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(30);

    EncoderConfig feederUpEnc = new EncoderConfig();

    feederUpEnc.positionConversionFactor(ENCODER_FEEDUP_POSITION_CONVERSION);

    feederUpEnc.velocityConversionFactor(ENCODER_FEEDUP_VELOCITY_CONVERSION);

    feedUpConfig.apply(feederUpEnc);

    feedUpMotor.configure(
        feedUpConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftLimitSwitch = new DigitalInput(indexerLeftLimitSwitchDIO);
    rightLimitSwitch = new DigitalInput(indexerRightLimitSwitchDIO);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.feedUpPercentOutput = feedUpMotor.get();
    inputs.feedUpVolts = feedUpMotor.getBusVoltage() * feedUpMotor.getAppliedOutput();
    inputs.feedUpAmps = feedUpMotor.getOutputCurrent();
    inputs.ballAtLeftSwitch = !leftLimitSwitch.get();
    inputs.ballAtRightSwitch = rightLimitSwitch.get();
  }

  @Override
  public void setPercent(double indexerPercent, double feedUpPercent) {
    feedUpMotor.set(feedUpPercent);
  }

  @Override
  public void setVoltage(double feedUpVolts) {
    feedUpMotor.setVoltage(feedUpVolts);
  }

  @Override
  public void stop() {
    feedUpMotor.set(0);
  }

  @Override
  public boolean isLeftSwitchPressed() {
    return !leftLimitSwitch.get();
  }

  @Override
  public boolean isRightSwitchPressed() {
    return rightLimitSwitch.get();
  }
}
