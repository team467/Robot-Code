package frc.robot.subsystems.climber;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.Relay;
import frc.robot.Schematic;
import org.littletonrobotics.junction.Logger;

public class ClimberIOSparkMax implements ClimberIO {

  private final CANSparkMax climberLeft;
  private final RelativeEncoder climberLeftEncoder;
  private final CANSparkMax climberRight;
  private final RelativeEncoder climberRightEncoder;
  private final Relay climberRatchet;
  private boolean ratchetLocked = false;
  private SparkLimitSwitch reverseLimitSwitchLeft;
  private SparkLimitSwitch fowardLimitSwitchLeft;
  private SparkLimitSwitch reverseLimitSwitchRight;
  private SparkLimitSwitch fowardLimitSwitchRight;

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
    climberLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
    climberRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
    climberRight.enableVoltageCompensation(12);
    climberLeft.enableVoltageCompensation(12);
    climberRight.setSmartCurrentLimit(80);
    climberLeft.setSmartCurrentLimit(80);
    reverseLimitSwitchLeft = climberLeft.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    fowardLimitSwitchLeft = climberLeft.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    reverseLimitSwitchRight =
        climberRight.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    fowardLimitSwitchRight =
        climberRight.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
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
    inputs.reverseLimitSwitchLeftPressed = reverseLimitSwitchLeft.isPressed();
    inputs.forwardLimitSwitchLeftPressed = fowardLimitSwitchLeft.isPressed();
    inputs.reverseLimitSwitchRightPressed = reverseLimitSwitchRight.isPressed();
    inputs.forwardLimitSwitchRightPressed = fowardLimitSwitchRight.isPressed();
  }

  @Override
  public void setMotorsOutputPercent(double percentOutput) {
    setLeftMotorPercentOutput(percentOutput);
    setRightMotorPercentOutput(percentOutput);
  }

  public void setLeftMotorPercentOutput(double percentOutput) {
    //    if ((climberLeft.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed()
    //            ||
    // climberLeft.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed())
    //        && percentOutput < 0) {
    //      climberLeft.set(0);
    //    } else {
    climberLeft.set(percentOutput);
    Logger.recordOutput("Climber/LeftPercentOutput", percentOutput);
    //    }
  }

  public void setRightMotorPercentOutput(double percentOutput) {
    //    if ((climberRight.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed()
    //            ||
    // climberRight.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed())
    //        && percentOutput < 0) {
    //      climberRight.set(0);
    //    } else {
    climberRight.set(percentOutput);
    Logger.recordOutput("Climber/RightPercentOutput", percentOutput);
    //    }
  }

  public void setRatchetLocked(boolean locked) {
    climberRatchet.set(ratchetLocked ? Relay.Value.kOff : Relay.Value.kOn);
    ratchetLocked = locked;
  }

  @Override
  public boolean getLimitSwitchLeft() {
    return fowardLimitSwitchLeft.isPressed() || reverseLimitSwitchLeft.isPressed();
  }

  @Override
  public boolean getLimitSwitchRight() {
    return fowardLimitSwitchRight.isPressed() || reverseLimitSwitchRight.isPressed();
  }
}
