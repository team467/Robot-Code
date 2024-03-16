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
  private SparkLimitSwitch reverseLimitSwitchLeft;
  private SparkLimitSwitch fowardLimitSwitchLeft;
  private SparkLimitSwitch reverseLimitSwitchRight;
  private SparkLimitSwitch fowardLimitSwitchRight;

  public ClimberIOSparkMax() {
    // Motors and Encoders
    climberLeft = new CANSparkMax(Schematic.CLIMBER_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    climberLeft.setInverted(true);
    climberLeftEncoder = climberLeft.getEncoder();
    climberLeftEncoder.setPositionConversionFactor(ClimberConstants.ROTS_TO_METERS);
    climberRight =
        new CANSparkMax(Schematic.CLIMBER_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
    climberRight.setInverted(true);
    climberRightEncoder = climberRight.getEncoder();
    climberRightEncoder.setPositionConversionFactor(ClimberConstants.ROTS_TO_METERS);
    climberRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
    climberLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
    climberRight.enableVoltageCompensation(12);
    climberLeft.enableVoltageCompensation(12);
    climberRight.setSmartCurrentLimit(80);
    climberLeft.setSmartCurrentLimit(80);
    // Limit Switches
    reverseLimitSwitchLeft = climberLeft.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    fowardLimitSwitchLeft = climberLeft.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    reverseLimitSwitchRight =
        climberRight.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    fowardLimitSwitchRight =
        climberRight.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    // Ratchet
    climberRatchet = new Relay(ClimberConstants.CLIMBER_RATCHET_ID, Relay.Direction.kReverse);
    climberRatchet.set(Relay.Value.kOff);
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
    inputs.ratchetLocked = climberRatchet.get().equals(Relay.Value.kOff);
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
    climberLeft.set(percentOutput);
    Logger.recordOutput("Climber/LeftPercentOutput", percentOutput);
  }

  public void setRightMotorPercentOutput(double percentOutput) {
    climberRight.set(percentOutput);
    Logger.recordOutput("Climber/RightPercentOutput", percentOutput);
  }

  public void setRatchetLocked(boolean locked) {
    climberRatchet.set(locked ? Relay.Value.kOff : Relay.Value.kOn);
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
