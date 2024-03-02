package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj.Relay;
import frc.robot.Schematic;

public class ClimberIOSparkMax implements ClimberIO {

  private final CANSparkMax climberLeft;
  private final RelativeEncoder climberLeftEncoder;
  private final CANSparkMax climberRight;
  private final RelativeEncoder climberRightEncoder;
  private final Relay climberRatchet;
  private boolean ratchetLocked = false;

  public ClimberIOSparkMax() {
    climberLeft = new CANSparkMax(Schematic.CLIMBER_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    climberLeft.setInverted(false);
    climberLeftEncoder = climberLeft.getEncoder();
    climberLeftEncoder.setPositionConversionFactor(ClimberConstants.ROTS_TO_METERS);
    climberRight =
        new CANSparkMax(Schematic.CLIMBER_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
    climberRight.setInverted(false);
    climberRightEncoder = climberRight.getEncoder();
    climberRightEncoder.setPositionConversionFactor(ClimberConstants.ROTS_TO_METERS);
    climberRatchet = new Relay(ClimberConstants.CLIMBER_RATCHET_ID, Relay.Direction.kReverse);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVoltsLeft = climberLeft.getBusVoltage() * climberLeft.getAppliedOutput();
    inputs.currentAmpsLeft = climberLeft.getOutputCurrent();
    inputs.motorPercentOutput = (climberLeft.getAppliedOutput() * climberLeft.getBusVoltage()) / 12;
    inputs.ClimberLeftPosition = climberLeftEncoder.getPosition();
    inputs.appliedVoltsRight = climberRight.getBusVoltage() * climberRight.getAppliedOutput();
    inputs.currentAmpsRight = climberRight.getOutputCurrent();
    inputs.ClimberRightPosition = climberRightEncoder.getPosition();
    inputs.ratchetLocked = ratchetLocked;
    inputs.limitSwitchPressed =
        (climberLeft.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed()
            || climberLeft
                .getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed)
                .isPressed());
  }

  @Override
  public void setMotorsOutputPercent(double percentOutput, boolean forward) {
    setLeftMotorVolts(percentOutput * 12, forward);
    setRightMotorVolts(percentOutput * 12, forward);
  }

  public void setLeftMotorVolts(double volts, boolean forward) {
    if (climberLeft.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed()
        || climberLeft.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed()
            && forward) {
      climberLeft.set(0);
    } else {
      climberLeft.set(volts);
    }
  }

  public void setRightMotorVolts(double volts, boolean forward) {
    if (climberRight.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed()
        || climberRight.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed()
            && forward) {
      climberRight.set(0);
    } else {
      climberRight.set(volts);
    }
  }

  public void setRatchetLocked(boolean locked) {
    climberRatchet.set(ratchetLocked ? Relay.Value.kOff : Relay.Value.kOn);
    ratchetLocked = locked;
  }
}
