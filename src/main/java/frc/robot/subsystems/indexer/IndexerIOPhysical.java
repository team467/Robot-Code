package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.drive.DriveConstants;

public class IndexerIOPhysical implements IndexerIO {
  private final CANSparkMax indexer;
  private final RelativeEncoder indexerEncoder;
  private final DigitalInput indexerLimitSwitch;

  public IndexerIOPhysical() {
    indexer = new CANSparkMax(IndexerConstants.INDEXER_ID, MotorType.kBrushless);
    indexerEncoder = indexer.getEncoder();
    indexerLimitSwitch = new DigitalInput(IndexerConstants.INDEXER_LIMIT_SWITCH_ID);
    double rotsToMeters = Units.rotationsToRadians(1);
    double rotsToRads =
        Units.rotationsToRadians(1) * DriveConstants.TURN_GEAR_RATIO.getRotationsPerInput();

    indexerEncoder.setVelocityConversionFactor(rotsToRads);
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
}
