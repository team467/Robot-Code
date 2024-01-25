package frc.robot.subsystems.IntakeNote;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

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
