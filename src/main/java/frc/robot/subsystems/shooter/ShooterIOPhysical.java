package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class ShooterIOPhysical implements ShooterIO {
  private CANSparkMax shooterLeader =
      new CANSparkMax(ShooterConstants.SHOOTER_LEADER_ID, MotorType.kBrushless);
  private CANSparkMax shooterFollower =
      new CANSparkMax(ShooterConstants.SHOOTER_FOLLOWER_ID, MotorType.kBrushless);
  private CANSparkMax indexer = new CANSparkMax(ShooterConstants.INDEXER_ID, MotorType.kBrushless);

  private RelativeEncoder shooterLeaderEncoder = shooterLeader.getEncoder();
  private RelativeEncoder shooterFollowerEncoder = shooterFollower.getEncoder();
  private RelativeEncoder indexerEncoder = indexer.getEncoder();

  private ShooterIOPhysical() {
    shooterLeaderEncoder.setVelocityConversionFactor(rotsToRads);
    shooterFollowerEncoder.setVelocityConversionFactor(rotsToRads);
    indexerEncoder.setVelocityConversionFactor(rotsToRads);
  }

  double rotsToRads = Units.rotationsToRadians(1);

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterLeaderVelocityRadPerSec = shooterLeaderEncoder.getVelocity();
    inputs.shooterFollowerVelocityRadPerSec = shooterFollowerEncoder.getVelocity();
    inputs.indexerVelocityRadPerSec = indexerEncoder.getVelocity();
  }
}
