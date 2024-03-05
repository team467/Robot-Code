package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Relay;
import frc.robot.Schematic;

public class ClimberIOTalonSRX implements ClimberIO {

  private final TalonSRX motorLeader;
  private final TalonSRX motorFollower;
  private final Relay climberRatchet;
  private boolean ratchetLocked = false;

  public ClimberIOTalonSRX() {
    motorLeader = new TalonSRX(Schematic.CLIMBER_LEFT_ID);
    motorLeader.setInverted(false);
    motorFollower = new TalonSRX(Schematic.CLIMBER_RIGHT_ID);
    motorFollower.setInverted(false);
    motorFollower.follow(motorLeader);
    climberRatchet = new Relay(ClimberConstants.CLIMBER_RATCHET_ID, Relay.Direction.kReverse);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.motorPercentOutput = motorLeader.getMotorOutputPercent();
    inputs.motorVoltage = motorLeader.getMotorOutputVoltage();
    inputs.ratchetLocked = ratchetLocked;
  }

  @Override
  public void setMotorsOutputPercent(double percentOutput) {
    if ((motorLeader.isFwdLimitSwitchClosed() == 1 || motorLeader.isRevLimitSwitchClosed() == 1)
        && percentOutput < 1) {
      motorLeader.set(TalonSRXControlMode.PercentOutput, 0);
    } else {
      motorLeader.set(TalonSRXControlMode.PercentOutput, percentOutput);
    }
  }

  @Override
  public void setRatchetLocked(boolean locked) {
    climberRatchet.set(locked ? Relay.Value.kOff : Relay.Value.kOn);
    ratchetLocked = locked;
  }
}
