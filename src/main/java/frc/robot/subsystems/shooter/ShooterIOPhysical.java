package frc.robot.subsystems.shooter;

import com.revrobotics.spark.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Schematic;

public class
ShooterIOPhysical implements ShooterIO {
  private final CANSparkMax shooterLeader;
  private final CANSparkMax shooterFollower;

  private final RelativeEncoder shooterLeaderEncoder;
  private final RelativeEncoder shooterFollowerEncoder;
  double rotsToRads = Units.rotationsToRadians(1);

  public ShooterIOPhysical() {
    shooterLeader = new CANSparkMax(Schematic.SHOOTER_LEFT_ID, MotorType.kBrushless);
    shooterFollower = new CANSparkMax(Schematic.SHOOTER_RIGHT_ID, MotorType.kBrushless);
    shooterLeader = shooterLeft.getEncoder();
    shooterFollower = shooterRight.getEncoder();
    shooterLeader.setInverted(true);
    shooterFollower.setInverted(false);
    shooterLeader.setIdleMode(IdleMode.kBrake);
    shooterFollower.setIdleMode(IdleMode.kBrake);
    shooterLeaderEncoder.setVelocityConversionFactor(rotsToRads / 60);
    shooterFollowerEncoder.setVelocityConversionFactor(rotsToRads / 60);
    shooterLeaderEncoder.setPositionConversionFactor(rotsToRads);
    shooterFollowerEncoder.setPositionConversionFactor(rotsToRads);

    shooterLeader.setSmartCurrentLimit(35);
    shooterFollower.setSmartCurrentLimit(35);

    shooterLeader.addFollower(shooterFollower);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterLeaderVelocityRadPerSec = shooterLeaderEncoder.getVelocity();
    inputs.shooterFollowerVelocityRadPerSec = shooterFollowerEncoder.getVelocity();
    inputs.shooterLeaderAppliedVolts =
        shooterLeader.getBusVoltage() * shooterLeader.getAppliedOutput();
    inputs.shooterLeaderCurrentAmps = shooterLeader.getOutputCurrent();
    inputs.shooterFollowerAppliedVolts =
        shooterFollower.getAppliedOutput() * shooterFollower.getBusVoltage();
    inputs.shooterFollowerCurrentAmps = shooterFollower.getOutputCurrent();
  }

  public void setShooterVoltage(double volts) {
    shooterLeader.setVoltage(volts);
  }

  public void setShooterVelocity(double velocity) {
    shooterLeader.set(velocity);
    shooterFollower.set(velocity);
  }
}
