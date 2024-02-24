package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Relay;

public class ClimberIOSparkMax implements ClimberIO {

  private final CANSparkMax climberLeft;
  private final RelativeEncoder climberLeftEncoder;
  private final CANSparkMax climberRight;
  private final RelativeEncoder climberRightEncoder;
  private final Relay climberRatchet;
  private boolean ratchetLocked = false;

  public ClimberIOSparkMax(int canIDLeft, int canIDRight, int relayPort) {
    climberLeft = new CANSparkMax(canIDLeft, CANSparkLowLevel.MotorType.kBrushless);
    climberLeft.setInverted(false);
    climberLeftEncoder = climberLeft.getEncoder();
    climberLeftEncoder.setPositionConversionFactor(ClimberConstants.ROTS_TO_METERS);
    climberRight = new CANSparkMax(canIDRight, CANSparkLowLevel.MotorType.kBrushless);
    climberRight.setInverted(false);
    climberRightEncoder = climberRight.getEncoder();
    climberRightEncoder.setPositionConversionFactor(ClimberConstants.ROTS_TO_METERS);
    climberRatchet = new Relay(relayPort);
  }

  @Override
  public void updateInput(ClimberIOInputs inputs) {
    inputs.appliedVoltsLeft = climberLeft.getBusVoltage() * climberLeft.getAppliedOutput();
    inputs.currentAmpsLeft = climberLeft.getOutputCurrent();
    inputs.ClimberLeftPosition = climberLeftEncoder.getPosition();
    inputs.appliedVoltsRight = climberRight.getBusVoltage() * climberRight.getAppliedOutput();
    inputs.currentAmpsRight = climberRight.getOutputCurrent();
    inputs.ClimberRightPosition = climberRightEncoder.getPosition();
    inputs.ratchetLocked = ratchetLocked;
  }

  @Override
  public void setMotorsOutputPercent(double percentOutput) {
    climberLeft.set(percentOutput);
    climberRight.set(percentOutput);
  }

  public void setLeftMotorVolts(double volts) {
    climberLeft.set(volts);
  }

  public void setRightMotorVolts(double volts) {
    climberRight.set(volts);
  }

  public void setRatchetLocked(boolean locked) {
    climberRatchet.set(ratchetLocked ? Relay.Value.kOff : Relay.Value.kOn);
    ratchetLocked = locked;
  }
}
