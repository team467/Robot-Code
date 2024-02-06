package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ClimberIOTalonSRXRight implements ClimberIO {

  private final TalonSRX motorRight = new TalonSRX(0);

  @Override
  public void updateInput(ClimberIOInputs inputs) {
    inputs.leftMotorPercentOutput = motorRight.getMotorOutputPercent();
    // inputs.leftMotorVoltage = motorRight.getMotorOutputVoltage();
  }

  @Override
  public void setRightMotorOutputPercent(double percentOutput) {
    motorRight.set(TalonSRXControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void setRightMotorInverted() {
    motorRight.setInverted(InvertType.InvertMotorOutput);
  }
}
