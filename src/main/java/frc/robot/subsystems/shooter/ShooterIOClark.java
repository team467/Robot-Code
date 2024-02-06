package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Schematic;

public class ShooterIOClark implements ShooterIO {
  private final CANSparkMax shooterLeader;
  private final CANSparkMax shooterFollower;

  private final RelativeEncoder shooterLeaderEncoder;
  private final RelativeEncoder shooterFollowerEncoder;
  double rotsToRads = Units.rotationsToRadians(1);

  public ShooterIOClark() {
    shooterLeader = new CANSparkMax(Schematic.SHOOTER_LEADER_ID, MotorType.kBrushless);
    shooterFollower = new CANSparkMax(Schematic.SHOOTER_FOLLOWER_ID, MotorType.kBrushless);
    shooterLeaderEncoder = shooterLeader.getEncoder();
    shooterFollowerEncoder = shooterFollower.getEncoder();
    shooterLeader.setInverted(false);
    shooterFollower.setInverted(true);
    shooterLeader.setIdleMode(IdleMode.kBrake);
    shooterFollower.setIdleMode(IdleMode.kBrake);
    shooterLeaderEncoder.setVelocityConversionFactor(rotsToRads / 60);
    shooterFollowerEncoder.setVelocityConversionFactor(rotsToRads / 60);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterLeaderVelocityRadPerSec = shooterLeaderEncoder.getVelocity();
    inputs.shooterFollowerVelocityRadPerSec = shooterFollowerEncoder.getVelocity();
    inputs.shooterLeaderAppliedVolts =
        shooterLeader.getBusVoltage() * shooterLeader.getAppliedOutput();
    inputs.shooterLeaderCurrentAmps = shooterLeader.getOutputCurrent();
  }

  public void setShooterVoltage(double volts) {
    shooterLeader.setVoltage(volts);
    shooterFollower.setVoltage(volts);
  }
}
