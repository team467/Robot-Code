package frc.robot.subsystems.IntakeNote;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.controls.GearRatio;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeNoteIOPhysical implements IntakeNoteIO {
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder;
  private final GearRatio GEAR_RATIO = new GearRatio(18,28);

  // Initializes motor encoders and limit switch for intake.
  public IntakeNoteIOPhysical() {
    intakeMotor = new CANSparkMax(12, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setInverted(false);
    intakeMotor.enableVoltageCompensation(12);

    intakeEncoder = intakeMotor.getEncoder();
    intakeEncoder.setPositionConversionFactor(Units.rotationsToRadians(1) * GEAR_RATIO.getRotationsPerInput());
    intakeEncoder.setVelocityConversionFactor(
        Units.rotationsPerMinuteToRadiansPerSecond(1) * GEAR_RATIO.getRotationsPerInput());
  }

  // Sets speed to motor, speed range is [-1,1].
  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  // Implements updateInputs and updates all our essential IO values.
  public void updateInputs(IntakeNoteIOInputs intakeInputs) {
    intakeInputs.positionRads = intakeEncoder.getPosition();
    intakeInputs.velocityRadsPerSec = intakeEncoder.getVelocity();
    
    intakeInputs.appliedVolts = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
    intakeInputs.motorCurrentAmps = intakeMotor.getOutputCurrent();
  }
}
