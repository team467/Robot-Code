package frc.robot.subsystems.intake.extend;

import static frc.robot.Schematic.intakeExtendCanId;
import static frc.robot.Schematic.intakeMotorCanId;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_LIMIT_ID;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_POSITION_CONVERSION;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_VELOCITY_CONVERSION;
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
import frc.robot.subsystems.intake.extend.IntakeExtendIO;

public class IntakeExtendIOSparkMax implements IntakeExtendIO {

  private final SparkMax extendMotor;
  private final DigitalInput collapsedLimitSwitch;
  private final RelativeEncoder extendMotorEncoder;
  private final SparkClosedLoopController pidController;
  private double setPos = 0;
  private boolean usingPID = false;

  public IntakeExtendIOSparkMax() {
    extendMotor = new SparkMax(intakeExtendCanId, MotorType.kBrushless);

    var extendConfig = new SparkMaxConfig();
    extendConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(30)
        .closedLoop
        .pid(PID_P, PID_I, PID_D);

    EncoderConfig intakeEnc = new EncoderConfig();

    EncoderConfig extendEnc = new EncoderConfig();
    extendEnc.positionConversionFactor(EXTEND_POSITION_CONVERSION);
    extendEnc.velocityConversionFactor(EXTEND_VELOCITY_CONVERSION);
    extendConfig.apply(extendEnc);

    pidController = extendMotor.getClosedLoopController();
    extendMotorEncoder = extendMotor.getEncoder();

    extendMotor.configure(
        extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    collapsedLimitSwitch = new DigitalInput(EXTEND_LIMIT_ID);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.extendPercentOutput = extendMotor.get();
    inputs.extendVelocity = extendMotorEncoder.getVelocity();
    inputs.extendVolts = extendMotor.getAppliedOutput();
    inputs.extendAmps = extendMotor.getOutputCurrent();
    inputs.isCollapsed = collapsedLimitSwitch.get();
    inputs.getExtendPos = extendMotorEncoder.getPosition();
    inputs.atSetpoint = extendMotor.getClosedLoopController().isAtSetpoint() && usingPID;
    inputs.hasSetpoint = usingPID;
    inputs.setpointValue = usingPID ? extendMotor.getClosedLoopController().getSetpoint() : 0.0;
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
  public void resetExtendEncoder(double position) {
    extendMotorEncoder.setPosition(position);
  }
}
