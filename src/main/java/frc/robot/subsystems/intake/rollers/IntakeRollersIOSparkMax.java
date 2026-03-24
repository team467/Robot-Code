package frc.robot.subsystems.intake.rollers;

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
import frc.robot.subsystems.intake.rollers.IntakeRollersIO;

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
        .smartCurrentLimit(30);

    EncoderConfig intakeEnc = new EncoderConfig();
    intakeEnc.positionConversionFactor(INTAKE_POSITION_CONVERSION);
    intakeEnc.velocityConversionFactor(INTAKE_POSITION_CONVERSION);
    intakeConfig.apply(intakeEnc);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
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
