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
    indexer.setIdleMode(IdleMode.kCoast);
    indexer.setInverted(false);
    indexerEncoder = indexer.getEncoder();
    indexerLimitSwitch = indexer.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    indexerEncoder.setVelocityConversionFactor(
        Units.rotationsPerMinuteToRadiansPerSecond(1)
            * IndexerConstants.INDEXER_GEAR_RATIO.getRotationsPerInput());
    indexerEncoder.setPositionConversionFactor(
        Units.rotationsToRadians(1) * IndexerConstants.INDEXER_GEAR_RATIO.getRotationsPerInput());
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
