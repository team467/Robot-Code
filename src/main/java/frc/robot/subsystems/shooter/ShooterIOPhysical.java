package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Schematic;

public class ShooterIOPhysical implements ShooterIO {
  private final CANSparkMax shooterTop;
  private final CANSparkMax shooterBottom;

  private final RelativeEncoder shooterTopEncoder;
  private final RelativeEncoder shooterBottomEncoder;
  double rotsToRads = Units.rotationsToRadians(1);

  public ShooterIOPhysical() {
    shooterTop = new CANSparkMax(Schematic.SHOOTER_TOP_ID, MotorType.kBrushless);
    shooterBottom = new CANSparkMax(Schematic.SHOOTER_BOTTOM_ID, MotorType.kBrushless);
    shooterTopEncoder = shooterTop.getEncoder();
    shooterBottomEncoder = shooterBottom.getEncoder();
    shooterTop.setInverted(false);
    shooterBottom.setInverted(true);
    shooterTop.setIdleMode(IdleMode.kBrake);
    shooterBottom.setIdleMode(IdleMode.kBrake);
    shooterTopEncoder.setVelocityConversionFactor(rotsToRads / 60);
    shooterBottomEncoder.setVelocityConversionFactor(rotsToRads / 60);
    shooterTopEncoder.setPositionConversionFactor(rotsToRads);
    shooterBottomEncoder.setPositionConversionFactor(rotsToRads);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterTopVelocityRadPerSec = shooterTopEncoder.getVelocity();
    inputs.shooterBottomVelocityRadPerSec = shooterBottomEncoder.getVelocity();
    inputs.shooterTopAppliedVolts = shooterTop.getBusVoltage() * shooterTop.getAppliedOutput();
    inputs.shooterTopCurrentAmps = shooterTop.getOutputCurrent();
  }

  public void setShooterVoltage(double volts) {
    shooterTop.setVoltage(volts);
    shooterBottom.setVoltage(volts);
  }
}
