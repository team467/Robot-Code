package frc.robot.subsystems.intakerelease;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import pabeles.concurrency.IntObjectConsumer;

import com.revrobotics.RelativeEncoder;

public class IntakeReleaseIOPhysical implements IntakeReleaseIO {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final DigitalInput cubeLimitSwitch;
  private final DigitalInput coneLimitSwitch;
  public IntakeReleaseIOPhysical(int motorID, int cubeLimID, int coneLimID) {
    motor = new CANSparkMax(motorID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    motor.setInverted(true);
    motor.enableVoltageCompensation(12);
    cubeLimitSwitch = new DigitalInput(cubeLimID);
    coneLimitSwitch = new DigitalInput(coneLimID);
  }

  @Override
  public void setVelocity(double speed) {
    motor.set(speed);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void updateInputs(IntakeReleaseIOInputs inputs) {
    inputs.motorPosition = encoder.getPosition();
    inputs.motorVelocity = encoder.getVelocity();
    inputs.motorAppliedVolts = motor.getBusVoltage();
    inputs.motorCurrent = motor.getOutputCurrent();
    inputs.motorTemp = motor.getMotorTemperature();
    inputs.cubeLimitSwitch = cubeLimitSwitch.get();
    inputs.coneLimitSwitch = coneLimitSwitch.get();
  }
}
