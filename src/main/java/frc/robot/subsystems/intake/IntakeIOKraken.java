package frc.robot.subsystems.intake;

import static frc.lib.utils.PhoenixUtil.tryUntilOk;
import static frc.robot.Schematic.intakeExtendCanId;
import static frc.robot.Schematic.intakeMotorCanId;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_LIMIT_ID;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_POSITION_CONVERSION;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_VELOCITY_CONVERSION;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_INTAKE_MOTOR_CURRENT_LIMIT;
import static frc.robot.subsystems.intake.IntakeConstants.PID_D;
import static frc.robot.subsystems.intake.IntakeConstants.PID_I;
import static frc.robot.subsystems.intake.IntakeConstants.PID_P;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOKraken implements IntakeIO {

  private final TalonFX intakeMotor;
  private final SparkMax extendMotor;
  private final DigitalInput collapsedLimitSwitch;
  private final RelativeEncoder extendMotorEncoder;
  private final SparkClosedLoopController pidController;
  private double setPos = 0;
  private boolean usingPID = false;

  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeCurrent;

  public IntakeIOKraken() {
    intakeMotor = new TalonFX(intakeMotorCanId);
    extendMotor = new SparkMax(intakeExtendCanId, MotorType.kBrushless);

    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.CurrentLimits.StatorCurrentLimit = INTAKE_INTAKE_MOTOR_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    var extendConfig = new SparkMaxConfig();
    extendConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(30)
        .closedLoop
        .pid(PID_P, PID_I, PID_D);

    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig));

    intakeAppliedVolts = intakeMotor.getMotorVoltage();
    intakeCurrent = intakeMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, intakeAppliedVolts, intakeCurrent);

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
    BaseStatusSignal.refreshAll(intakeAppliedVolts, intakeCurrent);

    inputs.extendPercentOutput = extendMotor.get();
    inputs.extendVelocity = extendMotorEncoder.getVelocity();
    inputs.intakeVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.extendVolts = extendMotor.getAppliedOutput();
    inputs.intakeAmps = intakeCurrent.getValueAsDouble();
    inputs.extendAmps = extendMotor.getOutputCurrent();
    inputs.isCollapsed = collapsedLimitSwitch.get();
    inputs.getExtendPos = extendMotorEncoder.getPosition();
    inputs.hasSetpoint = usingPID;
    inputs.setpointValue = usingPID ? extendMotor.getClosedLoopController().getSetpoint() : 0.0;
    inputs.atSetpoint = extendMotor.getClosedLoopController().isAtSetpoint();
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

  @Override
  public void setPIDEnabled(boolean enabled) {
    this.usingPID = enabled;
  }

  @Override
  public boolean getPIDEnabled() {
    return usingPID;
  }

  @Override
  public void resetExtendEncoder(double position) {
    extendMotorEncoder.setPosition(position);
  }
}
