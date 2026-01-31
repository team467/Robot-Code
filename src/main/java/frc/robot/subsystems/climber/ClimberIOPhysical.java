package frc.robot.subsystems.climber;

import static frc.lib.utils.PhoenixUtil.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIOPhysical implements ClimberIO {
  private final TalonFX talon;
  private final DigitalInput limitSwitch;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0);

  private boolean isCalibrated = false;
  private double targetRotation;

  public ClimberIOPhysical() {
    talon = new TalonFX(CLIMBER_MOTOR_ID);
    limitSwitch = new DigitalInput(LIMIT_SWITCH_ID);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = CLIMBER_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Slot0.kP = CLIMBER_KP;
    config.Slot0.kI = CLIMBER_KI;
    config.Slot0.kD = CLIMBER_KD;

    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
    tryUntilOk(5, () -> talon.setPosition(STARTING_DEGREES / 360.0));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    current = talon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

    inputs.positionDegrees = Units.rotationsToDegrees(position.getValueAsDouble());
    inputs.velocityDegreesPerSec = Units.rotationsToDegrees(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.targetRotation = targetRotation;
    inputs.atTargetRotation =
        isCalibrated && Math.abs(targetRotation - inputs.positionDegrees) < TOLERANCE;
    inputs.limitSwitch = !limitSwitch.get();

    if (inputs.limitSwitch) {
      this.isCalibrated = true;
      tryUntilOk(5, () -> talon.setPosition(CALIBRATION_POSITION_DEGREES / 360.0));
    }
    inputs.isCalibrated = this.isCalibrated;
  }

  @Override
  public void setPercent(double percent) {
    talon.setControl(voltageRequest.withOutput(percent * 12.0));
  }

  @Override
  public void setRotation(double degrees) {
    this.targetRotation = degrees;
  }

  @Override
  public void goToRotation() {
    if (!isCalibrated) {
      setPercent(CALIBRATION_PERCENT);
    } else {
      talon.setControl(positionRequest.withPosition(targetRotation / 360.0));
    }
  }
}
