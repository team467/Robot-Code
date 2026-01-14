import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Schematic;

public class ShooterIOPhysical implements ShooterIO {
  private final CANSparkMax shooterLeft;
  private final CANSparkMax shooterRight;

  private final RelativeEncoder shooterLeftEncoder;
  private final RelativeEncoder shooterRightEncoder;
  double rotsToRads = Units.rotationsToRadians(1);

  public ShooterIOPhysical() {
    shooterLeft = new CANSparkMax(Schematic.SHOOTER_LEFT_ID, MotorType.kBrushless);
    shooterRight = new CANSparkMax(Schematic.SHOOTER_RIGHT_ID, MotorType.kBrushless);
    shooterLeftEncoder = shooterLeft.getEncoder();
    shooterRightEncoder = shooterLeft.getEncoder();
    shooterLeft.setInverted(true);
    shooterRight.setInverted(false);
    shooterLeft.setIdleMode(IdleMode.kBrake);
    shooterRight.setIdleMode(IdleMode.kBrake);
    shooterLeftEncoder.setVelocityConversionFactor(rotsToRads / 60);
    shooterRightEncoder.setVelocityConversionFactor(rotsToRads / 60);
    shooterLeftEncoder.setPositionConversionFactor(rotsToRads);
    shooterRightEncoder.setPositionConversionFactor(rotsToRads);

    shooterLeft.setSmartCurrentLimit(35);
    shooterRight.setSmartCurrentLimit(35);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterLeftVelocityRadPerSec = shooterLeftEncoder.getVelocity();
    inputs.shooterRightVelocityRadPerSec = shooterLeftEncoder.getVelocity();
    inputs.shooterLeftAppliedVolts = shooterLeft.getBusVoltage() * shooterLeft.getAppliedOutput();
    inputs.shooterLeftCurrentAmps = shooterLeft.getOutputCurrent();
    inputs.shooterRightAppliedVolts =
        shooterRight.getAppliedOutput() * shooterRight.getBusVoltage();
    inputs.shooterRightCurrentAmps = shooterRight.getOutputCurrent();
  }

  public void setShooterVoltage(double volts) {
    shooterLeft.setVoltage(volts);
    shooterRight.setVoltage(volts);
  }

  public void setShooterVelocity(double velocity) {
    shooterLeft.set(velocity);
    shooterRight.set(velocity);
  }
}