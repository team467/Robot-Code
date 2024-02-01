package frc.robot.subsystems.IntakeNote;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeNoteIOPhysical implements IntakeNoteIO {
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakEncoder;

  public IntakeNoteIOPhysical(int motorID) {
    intakeMotor = new CANSparkMax(motorID, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setInverted(false);
    intakeMotor.enableVoltageCompensation(12);
    intakEncoder = intakeMotor.getEncoder();
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void updateInputs(IntakeNoteIOInputs intakeInputs) {
    intakeInputs.appliedVolts = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
    intakeInputs.motorVelocity = intakEncoder.getVelocity();
    intakeInputs.motorCurrent = intakeMotor.getOutputCurrent();
    intakeInputs.motorPosition = intakEncoder.getPosition();
  }
}
