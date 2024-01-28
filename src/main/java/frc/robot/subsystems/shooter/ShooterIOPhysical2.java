package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class ShooterIOPhysical2 implements ShooterIO {
  private final CANSparkMax shooter;

  private final RelativeEncoder shooterEncoder;
  double rotsToRads = Units.rotationsToRadians(1) * (ShooterConstants.WHEEL_DIAMETER / 2);

  public ShooterIOPhysical2() {
    shooter = new CANSparkMax(ShooterConstants.SHOOTER_2_ID, MotorType.kBrushless);
    shooter.setInverted(true);
    shooter.setIdleMode(IdleMode.kBrake);
    shooterEncoder = shooter.getEncoder();

    shooterEncoder.setVelocityConversionFactor(rotsToRads);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterLeaderVelocityRadPerSec = shooterEncoder.getVelocity();
    inputs.shooterLeaderAppliedVolts = shooter.getAppliedOutput();
  }

  public void setShooterVoltage(double volts) {
    shooter.setVoltage(volts);
  }
}
