package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class ArmIOPhysical implements ArmIO {

  private final CANSparkMax extendMotor;
  private final RelativeEncoder extendEncoder;
  private final SparkMaxLimitSwitch extendForwardLimitSwitchUnused;
  private final SparkMaxLimitSwitch extendReverseLimitSwitch;
  private final SparkMaxLimitSwitch rotateHighLimitSwitch;
  private final SparkMaxLimitSwitch rotateLowLimitSwitch;
  private final Relay ratchetSolenoid;

  private final CANSparkMax rotateMotor;
  private final RelativeEncoder rotateEncoder;

  private boolean ratchetLocked = false;

  public ArmIOPhysical(int extendMotorId, int rotateMotorId, int ratchetSolenoidId) {
    ratchetSolenoid = new Relay(ratchetSolenoidId);

    extendMotor = new CANSparkMax(extendMotorId, MotorType.kBrushless);
    rotateMotor = new CANSparkMax(rotateMotorId, MotorType.kBrushless);
    rotateHighLimitSwitch =
        rotateMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    rotateLowLimitSwitch =
        rotateMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    extendForwardLimitSwitchUnused =
        extendMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    extendReverseLimitSwitch =
        extendMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    extendEncoder = extendMotor.getEncoder();
    extendEncoder.setPositionConversionFactor(RobotConstants.get().armExtendConversionFactor());

    rotateEncoder = rotateMotor.getEncoder();
    rotateEncoder.setPositionConversionFactor(RobotConstants.get().armRotateConversionFactor());

    // Invert motors
    extendMotor.setInverted(true);
    rotateMotor.setInverted(false);

    extendMotor.enableVoltageCompensation(12);
    rotateMotor.enableVoltageCompensation(12);

    extendMotor.setIdleMode(IdleMode.kBrake);
    rotateMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.extendVelocity = extendEncoder.getVelocity();
    inputs.extendPosition = extendEncoder.getPosition();
    inputs.extendAppliedVolts = extendMotor.getBusVoltage() * extendMotor.getAppliedOutput();
    inputs.extendCurrent = extendMotor.getOutputCurrent();
    inputs.extendTemp = extendMotor.getMotorTemperature();
    inputs.extendForwardLimitSwitchUnused = extendForwardLimitSwitchUnused.isPressed();
    inputs.extendReverseLimitSwitch = extendReverseLimitSwitch.isPressed();
    inputs.rotateHighLimitSwitch = rotateHighLimitSwitch.isPressed();
    inputs.rotateLowLimitSwitch = rotateLowLimitSwitch.isPressed();
    inputs.rotatePosition = rotateEncoder.getPosition();
    inputs.rotateVelocity = rotateEncoder.getVelocity();
    inputs.ratchetLocked = ratchetLocked;
  }

  @Override
  public void setExtendVelocity(double velocity) {
    setRatchetLocked(velocity == 0);
    extendMotor.set(velocity);
  }

  @Override
  public void setExtendVoltage(double volts) {
    setRatchetLocked(volts == 0);
    extendMotor.setVoltage(volts);
  }

  @Override
  public void setRotateVelocity(double velocity) {
    Logger.getInstance().recordOutput("Rotate Velocity", velocity);
    rotateMotor.set(velocity);
  }

  @Override
  public void setRotateVoltage(double volts) {
    rotateMotor.setVoltage(volts);
  }

  @Override
  public void resetExtendEncoderPosition() {
    extendEncoder.setPosition(0);
  }

  @Override
  public void resetRotateEncoderPosition() {
    rotateEncoder.setPosition(0);
  }

  @Override
  public void setRatchetLocked(boolean locked) {
    ratchetSolenoid.set(locked ? Value.kOff : Value.kOn);
    ratchetLocked = locked;
  }
}
