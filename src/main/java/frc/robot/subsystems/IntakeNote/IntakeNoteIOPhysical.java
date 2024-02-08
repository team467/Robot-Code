package frc.robot.subsystems.intakeNote;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Schematic;

public class IntakeNoteIOPhysical implements IntakeNoteIO {
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder;

  public IntakeNoteIOPhysical() {
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
  }

  // Sets speed to motor, speed range is [-1,1].
  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void updateInputs(IntakeNoteIOInputs intakeInputs) {
    intakeInputs.positionRads = intakeEncoder.getPosition();
    intakeInputs.velocityRadsPerSec = intakeEncoder.getVelocity();

    intakeInputs.appliedVolts = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
    intakeInputs.motorCurrentAmps = intakeMotor.getOutputCurrent();
  }
}
