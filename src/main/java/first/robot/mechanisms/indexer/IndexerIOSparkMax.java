package first.robot.mechanisms.indexer;

import static first.robot.Schematic.indexerFeedupCanId;
import static first.robot.mechanisms.indexer.IndexerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IndexerIOSparkMax implements IndexerIO {
  private final SparkMax feedUpMotor;

  public IndexerIOSparkMax() {
    feedUpMotor = new SparkMax(CAN_PORT, indexerFeedupCanId, MotorType.kBrushless);

    var feedUpConfig = new SparkMaxConfig();
    feedUpConfig
        .inverted(FEEDUP_INVERTED)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(FEEDUP_CURRENT_LIMIT);
    feedUpMotor.configure(
        feedUpConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    var appliedOutput = feedUpMotor.getAppliedOutput();
    var busVoltage = feedUpMotor.getBusVoltage();
    var current = feedUpMotor.getOutputCurrent();
    inputs.connected = appliedOutput.isValid() && busVoltage.isValid() && current.isValid();
    inputs.feedUpPercentOutput = appliedOutput.get();
    inputs.feedUpVolts = busVoltage.get() * appliedOutput.get();
    inputs.feedUpAmps = current.get();
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {
    switch (outputs.mode) {
      case VOLTAGE -> feedUpMotor.setVoltage(outputs.targetVolts);
      case STOP -> feedUpMotor.stopMotor();
    }
  }
}
