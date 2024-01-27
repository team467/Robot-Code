package frc.robot.subsystems.IntakeNote;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeNoteIOBrushless implements IntakeNoteIO {
  private final CANSparkMax motor;
  private final RelativeEncoder intakEncoder;

  //private final RelativeEncoder intakeEncoder;

  public IntakeNoteIOBrushless(int motorID) {
    motor = new CANSparkMax(motorID, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(false);
    motor.enableVoltageCompensation(12);
    intakEncoder = motor.getEncoder();
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public void updateInputs(IntakeNoteIOInputs inputs) {
    inputs.appliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
    inputs.motorVelocity = intakEncoder.getVelocity();
  }
}
