package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.ENCODER_POSITION_CONVERSION;
import static frc.robot.subsystems.intake.IntakeConstants.ENCODER_VELOCITY_CONVERSION;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_EXTEND_ID;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_MOTOR_ID;

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

  public IntakeIOSparkMax() {
    intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushed);
    extendMotor = new SparkMax(INTAKE_EXTEND_ID, MotorType.kBrushed);

    var config = new SparkMaxConfig();
    config.inverted(false).idleMode(IdleMode.kBrake).voltageCompensation(12).smartCurrentLimit(30);

    EncoderConfig enc = new EncoderConfig();
    enc.positionConversionFactor(ENCODER_POSITION_CONVERSION);
    enc.velocityConversionFactor(ENCODER_VELOCITY_CONVERSION);
    config.apply(enc);

    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    extendMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    extendedInput = new DigitalInput(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.percentOutput = intakeMotor.get();
    inputs.volts = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
    inputs.amps = intakeMotor.getOutputCurrent();
    inputs.isExtended = isHopperExtended();
  }

  @Override
  public void setPercent(double percent) {
    intakeMotor.set(percent);
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    intakeMotor.set(0);
  }

  @Override
  public boolean isHopperExtended() {
    return extendedInput.get();
  }
}
