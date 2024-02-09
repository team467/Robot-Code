package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Schematic;

public class IndexerIOPhysical implements IndexerIO {
  private final CANSparkMax indexer;
  private final RelativeEncoder indexerEncoder;
  private final DigitalInput indexerLimitSwitchLeft;
  private final DigitalInput indexerLimitSwitchRight;

  public IndexerIOPhysical() {
    indexer = new CANSparkMax(Schematic.INDEXER_ID, MotorType.kBrushless);
    indexer.setIdleMode(IdleMode.kBrake);
    indexer.setInverted(true);
    indexerEncoder = indexer.getEncoder();
    indexerLimitSwitchLeft = new DigitalInput(IndexerConstants.INDEXER_LIMIT_SWITCH_LEFT_ID);
    indexerLimitSwitchRight = new DigitalInput(IndexerConstants.INDEXER_LIMIT_SWITCH_RIGHT_ID);
    double rotsToRads = Units.rotationsToRadians(1);

    indexerEncoder.setVelocityConversionFactor(rotsToRads / 60);
    indexerEncoder.setPositionConversionFactor(rotsToRads);
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerVelocityRadPerSec = indexerEncoder.getVelocity();
    inputs.indexerLimitSwitchLeftPressed =
        indexerLimitSwitchLeft.get();
    inputs.indexerLimitSwitchRightPressed = indexerLimitSwitchRight.get();
    inputs.indexerAppliedVolts = indexer.getAppliedOutput();
    inputs.indexerCurrentAmps = indexer.getOutputCurrent();
  }

  public void setIndexerVoltage(double volts) {
    indexer.setVoltage(volts);
  }

  public void setIndexerPercentVelocity(double percent) {
    indexer.set(percent);
  }
}
