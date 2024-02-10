package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ClimberIOTalonSRX implements ClimberIO {

  private final TalonSRX motor;

  public ClimberIOTalonSRX(int canID) {
    motor = new TalonSRX(canID);
    motor.setInverted(false);
  }

  @Override
  public void updateInput(ClimberIOInputs inputs) {
    inputs.motorPercentOutput = motor.getMotorOutputPercent();
    inputs.motorVoltage = motor.getMotorOutputVoltage();
  }

  @Override
  public void setMotorOutputPercent(double percentOutput) {
    motor.set(TalonSRXControlMode.PercentOutput, percentOutput);
  }
}
