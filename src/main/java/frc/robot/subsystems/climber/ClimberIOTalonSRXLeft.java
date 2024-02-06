package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ClimberIOTalonSRXLeft implements ClimberIO {

  private final TalonSRX motorLeft = new TalonSRX(0);

  @Override
  public void updateInput(ClimberIOInputs inputs) {
    inputs.leftMotorPercentOutput = motorLeft.getMotorOutputPercent();
    // inputs.leftMotorVoltage = motorLeft.getMotorOutputVoltage();
  }

  @Override
  public void setLeftMotorOutputPercent(double percentOutput) {
    motorLeft.set(TalonSRXControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void setLeftMotorInverted() {
    motorLeft.setInverted(InvertType.InvertMotorOutput);
  }
}
