package frc.robot.subsystems.intakerelease;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;

public class IntakeReleaseIOPhysical implements IntakeReleaseIO {
  private final TalonSRX motor;
  private final DigitalInput cubeLimitSwitch;
  private final ErrorCode coneLimitSwitch;

  public IntakeReleaseIOPhysical(int motorID, int cubeLimID) {
    motor = new TalonSRX(motorID);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setInverted(false);
    motor.enableVoltageCompensation(true);
    motor.configVoltageCompSaturation(12, 0);
    cubeLimitSwitch = new DigitalInput(cubeLimID);
    coneLimitSwitch = motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
  }

  @Override
  public void setPercent(double percent) {
    motor.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void setVoltage(double volts) {
    motor.set(ControlMode.PercentOutput, volts/12);
  }

  @Override
  public void updateInputs(IntakeReleaseIOInputs inputs, Wants wants) {
    inputs.motorPosition = motor.getSelectedSensorPosition();
    inputs.motorVelocity = motor.getSelectedSensorVelocity();
    inputs.motorCurrent = motor.getStatorCurrent();
    inputs.motorAppliedVolts = motor.getBusVoltage() * motor.getMotorOutputVoltage();
    inputs.cubeLimitSwitch = !cubeLimitSwitch.get();
    inputs.coneLimitSwitch = motor.isRevLimitSwitchClosed()==1;
    inputs.wantsCone = wants == Wants.CONE;
    inputs.wantsCube = wants == Wants.CUBE;
  }
}
