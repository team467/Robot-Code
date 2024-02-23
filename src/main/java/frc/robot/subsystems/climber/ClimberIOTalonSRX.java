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
    inputs.limitSwitchPressed = (motor.isFwdLimitSwitchClosed() == 1
        || motor.isRevLimitSwitchClosed() == 1);
  }

  @Override
  public void setMotorOutputPercent(double percentOutput) {
    if (motor.isFwdLimitSwitchClosed() == 1 || motor.isRevLimitSwitchClosed() == 1
      && percentOutput < 0) {
      motor.set(TalonSRXControlMode.PercentOutput, 0.0);  
    } else {
      motor.set(TalonSRXControlMode.PercentOutput, percentOutput);
    }
  }
}
