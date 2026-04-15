package frc.robot.subsystems.intake.extend;

import static frc.robot.Schematic.intakeExtendCanId;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_LIMIT_ID;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_POSITION_CONVERSION;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_VELOCITY_CONVERSION;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_EXTEND_MOTOR_CURRENT_LIMIT;
import static frc.robot.subsystems.intake.IntakeConstants.PID_D;
import static frc.robot.subsystems.intake.IntakeConstants.PID_I;
import static frc.robot.subsystems.intake.IntakeConstants.PID_P;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
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

public class IntakeExtendIOSparkMax implements IntakeExtendIO {

  private final SparkMax extendMotor;
  private final DigitalInput collapsedLimitSwitch;
  private final RelativeEncoder extendMotorEncoder;
  private final SparkClosedLoopController pidController;
  private double setPos = 0;
  private boolean usingPID = false;

  public IntakeExtendIOSparkMax() {
    extendMotor = new SparkMax(intakeExtendCanId, MotorType.kBrushless);

    pidController = extendMotor.getClosedLoopController();
    extendMotorEncoder = extendMotor.getEncoder();

    extendMotor.configure(
        getSparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    collapsedLimitSwitch = new DigitalInput(EXTEND_LIMIT_ID);
  }

  public SparkMaxConfig getSparkMaxConfig() {
    EncoderConfig extendEnc = new EncoderConfig();
    extendEnc.positionConversionFactor(EXTEND_POSITION_CONVERSION);
    extendEnc.velocityConversionFactor(EXTEND_VELOCITY_CONVERSION);
    var extendConfig = new SparkMaxConfig();
    extendConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12)
        .smartCurrentLimit((int) Math.round(INTAKE_EXTEND_MOTOR_CURRENT_LIMIT))
        .closedLoop
        .pid(PID_P, PID_I, PID_D)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    extendConfig.encoder.apply(extendEnc);
    return extendConfig;
  }

  @Override
  public void updateInputs(IntakeExtendIOInputs inputs) {
    inputs.extendPercentOutput = extendMotor.get();
    inputs.extendVelocity = extendMotorEncoder.getVelocity();
    inputs.extendVolts = extendMotor.getAppliedOutput();
    inputs.extendAmps = extendMotor.getOutputCurrent();
    inputs.isCollapsed = isCollapsed();
    inputs.getExtendPos = extendMotorEncoder.getPosition();
    inputs.atSetpoint = extendMotor.getClosedLoopController().isAtSetpoint() && usingPID;
    inputs.hasSetpoint = usingPID;
    inputs.setpointValue = extendMotor.getClosedLoopController().getSetpoint();
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
  public void setPercentExtend(double extendPercent) {
    extendMotor.set(extendPercent);
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

  public void setPIDEnabled(boolean enabled) {
    this.usingPID = enabled;
  }

  @Override
  public void setIdleMode(boolean coast) {
    if (coast) {
      extendMotor.configure(
          getSparkMaxConfig().idleMode(IdleMode.kCoast),
          ResetMode.kResetSafeParameters,
          PersistMode.kNoPersistParameters);
    } else {
      extendMotor.configure(
          getSparkMaxConfig().idleMode(IdleMode.kBrake),
          ResetMode.kResetSafeParameters,
          PersistMode.kNoPersistParameters);
    }
  }

  @Override
  public void resetExtendEncoder(double position) {
    extendMotorEncoder.setPosition(position);
  }
}
