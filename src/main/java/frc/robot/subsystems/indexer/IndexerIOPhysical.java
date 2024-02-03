package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOPhysical implements IndexerIO {
  private final CANSparkMax indexer;
  private final RelativeEncoder indexerEncoder;
  private final DigitalInput indexerLimitSwitch;

  public IndexerIOPhysical() {
    indexer = new CANSparkMax(IndexerConstants.INDEXER_ID, MotorType.kBrushless);
    indexer.setIdleMode(IdleMode.kBrake);
    indexer.setInverted(true);
    indexerEncoder = indexer.getEncoder();
    indexerLimitSwitch = new DigitalInput(IndexerConstants.INDEXER_LIMIT_SWITCH_ID);
    double rotsToMeters = Units.rotationsToRadians(1) * (IndexerConstants.WHEEL_DIAMETER / 2);
    double rotsToRads = Units.rotationsToRadians(1);

    indexerEncoder.setVelocityConversionFactor(rotsToRads / 60);
    indexerEncoder.setPositionConversionFactor(rotsToMeters);
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerVelocityRadPerSec = indexerEncoder.getVelocity();
    inputs.indexerVelocityMetersPerSec = indexerEncoder.getPosition();
    inputs.indexerLimitSwitchPressed = indexerLimitSwitch.get();
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
