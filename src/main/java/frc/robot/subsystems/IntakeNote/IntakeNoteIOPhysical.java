package frc.robot.subsystems.IntakeNote;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeNoteIOPhysical implements IntakeNoteIO {
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakEncoder;
  private final DigitalInput limitSwitch;

  public IntakeNoteIOPhysical(int motorID, int limitSwitchID) {
    intakeMotor = new CANSparkMax(motorID, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setInverted(false);
    intakeMotor.enableVoltageCompensation(12);
    intakEncoder = intakeMotor.getEncoder();
    limitSwitch = new DigitalInput(limitSwitchID);
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void updateInputs(IntakeNoteIOInputs intakeInputs) {
    intakeInputs.appliedVolts = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
    intakeInputs.motorVelocity = intakEncoder.getVelocity();
    intakeInputs.motorCurrent = intakeMotor.getOutputCurrent();
    intakeInputs.motorPosition = intakEncoder.getPosition();
    intakeInputs.hasNote = () -> limitSwitch.get();
  }
}
