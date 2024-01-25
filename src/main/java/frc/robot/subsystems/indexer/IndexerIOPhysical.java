package frc.robot.subsystems.indexer;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
public class IndexerIOPhysical implements IndexerIO {
  private final CANSparkMax indexer;
  private final SparkLimitSwitch indexerLimitSwitch;

  public IndexerIOPhysical() {
    indexer = new CANSparkMax(IndexerConstants.INDEXER_ID, MotorType.kBrushless);
    indexerLimitSwitch = indexer.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
  }

  public updateInputs(IndexerIOInputs inputs){
  }
}
