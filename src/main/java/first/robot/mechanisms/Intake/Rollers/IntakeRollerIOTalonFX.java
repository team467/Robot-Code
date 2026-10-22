package first.robot.mechanisms.Intake.Rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import first.robot.Schematic;
import first.robot.constants.IntakeConstants;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX rollerMotor;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temperature;

  public IntakeRollerIOTalonFX() {
    rollerMotor = new TalonFX(Schematic.intakeMotorCanId, Schematic.canBus);

    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.CurrentLimits.StatorCurrentLimit =
        IntakeConstants.INTAKE_ROLLERS_MOTOR_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerMotor.getConfigurator().apply(intakeConfig);

    position = rollerMotor.getPosition();
    velocity = rollerMotor.getVelocity();
    appliedVolts = rollerMotor.getMotorVoltage();
    supplyCurrent = rollerMotor.getSupplyCurrent();
    statorCurrent = rollerMotor.getStatorCurrent();
    temperature = rollerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrent, statorCurrent, temperature);
    rollerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, supplyCurrent, statorCurrent, temperature)
            .isOK();
    inputs.position = position.getValueAsDouble();
    inputs.velocity = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.temperatureCelsius = temperature.getValueAsDouble();
  }

  @Override
  public void applyOutputs(IntakeRollerIOOutputs outputs) {
    switch (outputs.mode) {
      case VOLTAGE -> rollerMotor.setVoltage(outputs.targetVolts);
      case STOP -> rollerMotor.stopMotor();
    }
  }
}
