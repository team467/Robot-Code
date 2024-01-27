package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeNoteIOBrushless implements IntakeNoteIO {
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakEncoder;

  public IntakeNoteIOBrushless(int motorID) {
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
