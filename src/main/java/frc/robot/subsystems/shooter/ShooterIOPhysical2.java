package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.util.Units;

public class ShooterIOPhysical2 implements ShooterIO {
  private final CANSparkMax shooter;

  private final RelativeEncoder shooterEncoder;

  public ShooterIOPhysical2() {
    shooter = new CANSparkMax(ShooterConstants.SHOOTER_2_ID, MotorType.kBrushless);
    shooterEncoder = shooter.getEncoder();

    shooterEncoder.setVelocityConversionFactor(rotsToRads);
  }

  double rotsToRads = Units.rotationsToRadians(1);

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterLeaderVelocityRadPerSec = shooterEncoder.getVelocity();
  }

  public void setShooterVoltage(double volts) {
    shooter.setVoltage(volts);
  }
}
