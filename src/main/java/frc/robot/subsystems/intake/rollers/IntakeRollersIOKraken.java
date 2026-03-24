package frc.robot.subsystems.intake.rollers;

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

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeRollersIOKraken implements IntakeRollersIO {

  private final TalonFX intakeMotor;
  private double setPos = 0;

  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeCurrent;

  public IntakeIOKraken() {
    intakeMotor = new TalonFX(intakeMotorCanId);

    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.CurrentLimits.StatorCurrentLimit = INTAKE_INTAKE_MOTOR_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig));

    intakeAppliedVolts = intakeMotor.getMotorVoltage();
    intakeCurrent = intakeMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, intakeAppliedVolts, intakeCurrent);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(intakeAppliedVolts, intakeCurrent);
    inputs.intakeVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeAmps = intakeCurrent.getValueAsDouble();
  }

  @Override
  public void setPercentIntake(double intakePercent) {
    intakeMotor.set(intakePercent);
  }

  @Override
  public void setVoltageIntake(double intakeVolts) {
    intakeMotor.setVoltage(intakeVolts);
  }
}