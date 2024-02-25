package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Relay;

public class ClimberIOTalonSRX implements ClimberIO {

  private final TalonSRX motor;
  private final Relay climberRatchet;
  private boolean ratchetLocked = false;

  public ClimberIOTalonSRX(int canID, int relayChannel) {
    motor = new TalonSRX(canID);
    motor.setInverted(false);
    climberRatchet = new Relay(relayChannel, Relay.Direction.kReverse);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.motorPercentOutput = motor.getMotorOutputPercent();
    inputs.motorVoltage = motor.getMotorOutputVoltage();
    inputs.ratchetLocked = ratchetLocked;
  }

  @Override
  public void setMotorOutputPercent(double percentOutput) {
    motor.set(TalonSRXControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void setRatchetLocked(boolean locked) {
    climberRatchet.set(locked ? Relay.Value.kOff : Relay.Value.kOn);
    ratchetLocked = locked;
  }
}
