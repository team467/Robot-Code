package frc.robot.subsystems.IntakeNote;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/* Implements: To access the interface methods, the interface must be "implemented"
(kinda like inherited) by another class with the implements keyword (instead of extends). */
public class IntakeNoteIOBrushless implements IntakeNoteIO {
  private final CANSparkMax motor;

  public IntakeNoteIOBrushless(int motorID) {
    motor = new CANSparkMax(motorID, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(false);
    motor.enableVoltageCompensation(12);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }
}
