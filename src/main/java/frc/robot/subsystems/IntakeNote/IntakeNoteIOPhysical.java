package frc.robot.subsystems.IntakeNote;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeNoteIOPhysical implements IntakeNoteIO {
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder;
  private final DigitalInput limitSwitch;

  // Initializes motor encoders and limit switch for intake.
  public IntakeNoteIOPhysical(int motorID, int limitSwitchID) {
    intakeMotor = new CANSparkMax(motorID, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setInverted(false);
    intakeMotor.enableVoltageCompensation(12);
    intakeEncoder = intakeMotor.getEncoder();
    limitSwitch = new DigitalInput(limitSwitchID);
  }

  // Sets speed to motor, speed range is [-1,1].
  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  // Implements updateInputs and updates all our essential IO values.
  public void updateInputs(IntakeNoteIOInputs intakeInputs) {
    intakeInputs.appliedVolts = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
    intakeInputs.motorVelocity = intakeEncoder.getVelocity();
    intakeInputs.motorCurrent = intakeMotor.getOutputCurrent();
    intakeInputs.motorPosition = intakeEncoder.getPosition();
    intakeInputs.hasNote = () -> limitSwitch.get();
  }
}
