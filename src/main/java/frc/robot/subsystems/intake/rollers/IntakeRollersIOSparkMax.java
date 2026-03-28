package frc.robot.subsystems.intake.rollers;

import static frc.robot.Schematic.intakeMotorCanId;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_EXTEND_MOTOR_CURRENT_LIMIT;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_POSITION_CONVERSION;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeRollersIOSparkMax implements IntakeRollersIO {

  private final SparkMax intakeMotor;
  private double setPos = 0;
  private boolean usingPID = false;

  public IntakeRollersIOSparkMax() {
    intakeMotor = new SparkMax(intakeMotorCanId, MotorType.kBrushless);

    var intakeConfig = new SparkMaxConfig();
    intakeConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12)
        .smartCurrentLimit((int) Math.round(INTAKE_EXTEND_MOTOR_CURRENT_LIMIT));

    EncoderConfig intakeEnc = new EncoderConfig();
    intakeEnc.positionConversionFactor(INTAKE_POSITION_CONVERSION);
    intakeEnc.velocityConversionFactor(INTAKE_POSITION_CONVERSION);
    intakeConfig.apply(intakeEnc);
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    inputs.intakePercentOutput = intakeMotor.get();
    inputs.intakeVolts = intakeMotor.getAppliedOutput();
    inputs.intakeAmps = intakeMotor.getOutputCurrent();
  }

  @Override
  public void setPercentIntake(double intakePercent) {
    intakeMotor.set(intakePercent);
  }

  @Override
  public void setVoltageIntake(double intakeVolts) {
    intakeMotor.setVoltage(intakeVolts);
  }

  public void setPIDEnabled(boolean enabled) {
    this.usingPID = enabled;
  }
}
