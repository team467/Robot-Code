package frc.robot.subsystems.effector;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.effector.Effector.Wants;

public class EffectorIOBrushed implements EffectorIO {
  private final CANSparkMax motor;
  private final DigitalInput cubeLimitSwitch;
  private final SparkMaxLimitSwitch coneLimitSwitch;

  public EffectorIOBrushed(int motorID, int cubeLimID) {
    motor = new CANSparkMax(motorID, MotorType.kBrushed);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(false);
    motor.enableVoltageCompensation(12);
    motor.setSmartCurrentLimit(40);
    cubeLimitSwitch = new DigitalInput(cubeLimID);
    coneLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void updateInputs(EffectorIOInputs inputs, Wants wants) {
    inputs.motorAppliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
    inputs.motorCurrent = motor.getOutputCurrent();
    inputs.cubeLimitSwitch = !cubeLimitSwitch.get();
    inputs.coneLimitSwitch = coneLimitSwitch.isPressed();
    inputs.wantsCone = wants == Wants.CONE;
    inputs.wantsCube = wants == Wants.CUBE;
  }
}
