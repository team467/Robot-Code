package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.RobotConstants;

public class ArmIOPhysical implements ArmIO {
  private final CANSparkMax extendMotor;
  private final RelativeEncoder extendEncoder;
  private final DigitalInput extendLimitSwitch;
  private final SparkMaxLimitSwitch rotateHighLimitSwitch;
  private final SparkMaxLimitSwitch rotateLowLimitSwitch;
  private final DigitalOutput ratchetSolenoid;

  private CANSparkMax rotateMotor;
  private RelativeEncoder rotateEncoder;

  public ArmIOPhysical(
      int extendMotorId, int rotateMotorId, int extendLimitSwitchId, int ratchetSolenoidId) {
    extendLimitSwitch = new DigitalInput(extendLimitSwitchId);

    ratchetSolenoid = new DigitalOutput(ratchetSolenoidId);

    extendMotor = new CANSparkMax(extendMotorId, MotorType.kBrushless);
    rotateMotor = new CANSparkMax(rotateMotorId, MotorType.kBrushless);
    rotateHighLimitSwitch =
        rotateMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    rotateLowLimitSwitch =
        rotateMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    extendEncoder = extendMotor.getEncoder();
    extendEncoder.setPositionConversionFactor(RobotConstants.get().armExtendConversionFactor());

    rotateEncoder = rotateMotor.getEncoder();
    rotateEncoder.setPositionConversionFactor(RobotConstants.get().armRotateConversionFactor());

    // Invert motors
    extendMotor.setInverted(false);
    rotateMotor.setInverted(false); // TODO: check if inverted

    extendMotor.enableVoltageCompensation(12);
    rotateMotor.enableVoltageCompensation(12);

    extendMotor.setIdleMode(IdleMode.kBrake);
    rotateMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.extendVelocity = extendEncoder.getVelocity();
    inputs.extendPosition = extendEncoder.getPosition();
    inputs.extendAppliedVolts = extendMotor.getBusVoltage();
    inputs.extendCurrent = extendMotor.getOutputCurrent();
    inputs.extendTemp = extendMotor.getMotorTemperature();
    inputs.extendLimitSwitch = extendLimitSwitch.get();
    inputs.rotateHighLimitSwitch = rotateHighLimitSwitch.isPressed();
    inputs.rotateLowLimitSwitch = rotateLowLimitSwitch.isPressed();
    inputs.rotatePosition = rotateEncoder.getPosition();
    inputs.rotateVelocity = rotateEncoder.getVelocity();
  }

  @Override
  public void setExtendVelocity(double velocity) {
    extendMotor.set(velocity);
  }

  @Override
  public void setRotateVelocity(double velocity) {
    rotateMotor.set(velocity);
  }

  @Override
  public void setExtendVoltage(double volts) {
    extendMotor.setVoltage(volts);
  }

  @Override
  public void setRotateVoltage(double volts) {
    rotateMotor.setVoltage(volts);
  }

  @Override
  public void resetEncoderPosition() {
    extendEncoder.setPosition(0);
  }

  @Override
  public void resetRotateEncoder() {
    rotateEncoder.setPosition(0);
  }

  @Override
  public void setRatchetLocked(boolean locked) {
    ratchetSolenoid.set(locked);
  }
}
