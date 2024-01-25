package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.util.Units;

public class ShooterIOPhysical implements ShooterIO {
  private final CANSparkMax shooterLeader;
  private final CANSparkMax shooterFollower;

  private final RelativeEncoder shooterLeaderEncoder;
  private final RelativeEncoder shooterFollowerEncoder;

  public ShooterIOPhysical() {
    shooterLeader = new CANSparkMax(ShooterConstants.SHOOTER_LEADER_ID, MotorType.kBrushless);
    shooterFollower = new CANSparkMax(ShooterConstants.SHOOTER_FOLLOWER_ID, MotorType.kBrushless);
    shooterLeaderEncoder = shooterLeader.getEncoder();
    shooterFollowerEncoder = shooterFollower.getEncoder();

    shooterLeaderEncoder.setVelocityConversionFactor(rotsToRads);
    shooterFollowerEncoder.setVelocityConversionFactor(rotsToRads);
  }

  double rotsToRads = Units.rotationsToRadians(1);

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterLeaderVelocityRadPerSec = shooterLeaderEncoder.getVelocity();
    inputs.shooterFollowerVelocityRadPerSec = shooterFollowerEncoder.getVelocity();
  }

  public void setShooterVoltage(double volts) {
    shooterLeader.setVoltage(volts);
    shooterFollower.follow(shooterLeader);
  }
}
