package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_LIMIT_ID;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_POSITION_CONVERSION;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_VELOCITY_CONVERSION;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_EXTEND_ID;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_MOTOR_ID;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_POSITION_CONVERSION;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSparkMax implements IntakeIO {

  private final SparkMax intakeMotor;
  private final SparkMax extendMotor;
  private final DigitalInput extendedInput;
  private final RelativeEncoder extendMotorEncoder;

  public IntakeIOSparkMax() {
    intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushed);
    extendMotor = new SparkMax(INTAKE_EXTEND_ID, MotorType.kBrushed);
    extendMotorEncoder = extendMotor.getEncoder();

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
        .smartCurrentLimit(30);

    EncoderConfig intakeEnc = new EncoderConfig();
    intakeEnc.positionConversionFactor(INTAKE_POSITION_CONVERSION);
    intakeEnc.velocityConversionFactor(INTAKE_POSITION_CONVERSION);
    intakeConfig.apply(intakeEnc);

    EncoderConfig extendEnc = new EncoderConfig();
    extendEnc.positionConversionFactor(EXTEND_POSITION_CONVERSION);
    extendEnc.velocityConversionFactor(EXTEND_VELOCITY_CONVERSION);
    extendConfig.apply(extendEnc);

    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    extendMotor.configure(
        extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    extendedInput = new DigitalInput(EXTEND_LIMIT_ID);
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
    inputs.isExtended = isHopperExtended();
  }

  @Override
  public void setPercentIntake(double intakePercent) {
    intakeMotor.set(intakePercent);
  }
  @Override
  public void setPercentExtend(double extendPercent) {
    intakeMotor.set(extendPercent);
  }

  @Override
  public void setVoltageIntake(double intakeVolts) {
    intakeMotor.setVoltage(intakeVolts);
  }
  @Override
  public void setVoltageExtend(double extendVolts){
    extendMotor.setVoltage(extendVolts);
  }

  @Override
  public boolean isHopperExtended() {
    return extendedInput.get();
  }
}
