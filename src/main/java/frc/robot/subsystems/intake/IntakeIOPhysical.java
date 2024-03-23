package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Schematic;

public class IntakeIOPhysical implements IntakeIO {
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder;

  public IntakeIOPhysical() {
    intakeMotor = new CANSparkMax(Schematic.INTAKE_ID, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setInverted(false);
    intakeMotor.enableVoltageCompensation(12);

    intakeEncoder = intakeMotor.getEncoder();
    intakeEncoder.setPositionConversionFactor(
        Units.rotationsToRadians(1) * IntakeConstants.GEAR_RATIO.getRotationsPerInput());
    intakeEncoder.setVelocityConversionFactor(
        Units.rotationsPerMinuteToRadiansPerSecond(1)
            * IntakeConstants.GEAR_RATIO.getRotationsPerInput());

    intakeMotor.setSmartCurrentLimit(40);
  }

  // Sets speed to motor, speed range is [-1,1].
  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void updateInputs(IntakeIOInputs intakeInputs) {
    intakeInputs.positionRads = intakeEncoder.getPosition();
    intakeInputs.velocityRadsPerSec = intakeEncoder.getVelocity();

    intakeInputs.appliedVolts = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
    intakeInputs.motorCurrentAmps = intakeMotor.getOutputCurrent();
  }
}
