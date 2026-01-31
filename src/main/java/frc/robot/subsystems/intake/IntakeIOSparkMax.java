package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_LIMIT_ID;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_POSITION_CONVERSION;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_VELOCITY_CONVERSION;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_EXTEND_ID;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_MOTOR_ID;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_POSITION_CONVERSION;
import static frc.robot.subsystems.intake.IntakeConstants.PID_D;
import static frc.robot.subsystems.intake.IntakeConstants.PID_I;
import static frc.robot.subsystems.intake.IntakeConstants.PID_P;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSparkMax implements IntakeIO {

  private final SparkMax intakeMotor;
  private final SparkMax extendMotor;
  private final DigitalInput collapsedLimitSwitch;
  private final RelativeEncoder extendMotorEncoder;
  private final SparkClosedLoopController pidController;
  private double setPos = 0;
  private boolean usingPID = false;

  public IntakeIOSparkMax() {
    intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
    extendMotor = new SparkMax(INTAKE_EXTEND_ID, MotorType.kBrushless);

    var intakeConfig = new SparkMaxConfig();
    intakeConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(30);
    var extendConfig = new SparkMaxConfig();
    extendConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(30)
        .closedLoop
        .pid(PID_P, PID_I, PID_D);

    EncoderConfig intakeEnc = new EncoderConfig();
    intakeEnc.positionConversionFactor(INTAKE_POSITION_CONVERSION);
    intakeEnc.velocityConversionFactor(INTAKE_POSITION_CONVERSION);
    intakeConfig.apply(intakeEnc);

    EncoderConfig extendEnc = new EncoderConfig();
    extendEnc.positionConversionFactor(EXTEND_POSITION_CONVERSION);
    extendEnc.velocityConversionFactor(EXTEND_VELOCITY_CONVERSION);
    extendConfig.apply(extendEnc);

    pidController = extendMotor.getClosedLoopController();
    extendMotorEncoder = extendMotor.getEncoder();

    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    extendMotor.configure(
        extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    collapsedLimitSwitch = new DigitalInput(EXTEND_LIMIT_ID);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakePercentOutput = intakeMotor.get();
    inputs.extendPercentOutput = extendMotor.get();
    inputs.extendVelocity = extendMotorEncoder.getVelocity();
    inputs.intakeVolts = intakeMotor.getAppliedOutput();
    inputs.extendVolts = extendMotor.getAppliedOutput();
    inputs.intakeAmps = intakeMotor.getOutputCurrent();
    inputs.extendAmps = extendMotor.getOutputCurrent();
    inputs.isCollapsed = collapsedLimitSwitch.get();
    inputs.getExtendPos = extendMotorEncoder.getPosition();
    inputs.setpointExtendPos = this.setPos;
  }

  @Override
  public void extendToPosition(double position) {
    pidController.setSetpoint(position, ControlType.kPosition);
  }

  @Override
  public boolean isCollapsed() {
    return collapsedLimitSwitch.get();
  }

  @Override
  public void setPercentIntake(double intakePercent) {
    intakeMotor.set(intakePercent);
  }

  @Override
  public void setPercentExtend(double extendPercent) {
    extendMotor.set(extendPercent);
  }

  @Override
  public void setVoltageIntake(double intakeVolts) {
    intakeMotor.setVoltage(intakeVolts);
  }

  @Override
  public void setVoltageExtend(double extendVolts) {
    extendMotor.setVoltage(extendVolts);
  }

  @Override
  public void goToPos(double setPos) {
    this.setPos = setPos;
    if (usingPID) {
      pidController.setSetpoint(setPos, ControlType.kPosition);
    }
  }

  @Override
  public boolean isHopperCollapsed() {
    return collapsedLimitSwitch.get();
  }

  public void setPIDEnabled(boolean enabled) {
    this.usingPID = enabled;
  }

  @Override
  public void resetExtendEncoder() {
    extendMotorEncoder.setPosition(0);
  }
}
