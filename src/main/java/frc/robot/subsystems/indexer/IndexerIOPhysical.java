package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.util.Units;
import frc.robot.Schematic;

public class IndexerIOPhysical implements IndexerIO {
  private final CANSparkMax indexer;
  private final RelativeEncoder indexerEncoder;
  private final SparkLimitSwitch indexerLimitSwitch;

  public IndexerIOPhysical() {
    indexer = new CANSparkMax(Schematic.INDEXER_ID, MotorType.kBrushless);
    indexer.setIdleMode(IdleMode.kBrake);
    indexer.setInverted(true);
    indexerEncoder = indexer.getEncoder();
    indexerLimitSwitch = indexer.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    double rotsToRads = Units.rotationsToRadians(1);

    indexerEncoder.setVelocityConversionFactor(rotsToRads / 60);
    indexerEncoder.setPositionConversionFactor(rotsToRads);
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerVelocityRadPerSec = indexerEncoder.getVelocity();
    inputs.indexerLimitSwitchPressed = indexerLimitSwitch.isPressed();
    inputs.indexerAppliedVolts = indexer.getAppliedOutput() * indexer.getBusVoltage();
    inputs.indexerCurrentAmps = indexer.getOutputCurrent();
  }

  public void setIndexerVoltage(double volts) {
    indexer.setVoltage(volts);
  }

  public void setIndexerPercentVelocity(double percent) {
    indexer.set(percent);
  }
}
