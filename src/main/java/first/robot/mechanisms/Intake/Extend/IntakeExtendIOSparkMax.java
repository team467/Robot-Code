package first.robot.mechanisms.Intake.Extend;

import static first.robot.constants.IntakeConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import first.robot.Schematic;
import org.wpilib.hardware.discrete.DigitalInput;

public class IntakeExtendIOSparkMax implements IntakeExtendIO {
  private final SparkMax extendMotor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final DigitalInput limitSwitch;

  public IntakeExtendIOSparkMax() {
    extendMotor =
        new SparkMax(
            Schematic.canPort, Schematic.intakeExtendCanId, SparkLowLevel.MotorType.kBrushless);

    var extendConfig = new SparkMaxConfig();
    extendConfig
        .inverted(false)
        .idleMode(SparkBaseConfig.IdleMode.kCoast)
        .voltageCompensation(12)
        .smartCurrentLimit((int) INTAKE_EXTEND_MOTOR_CURRENT_LIMIT)
        .closedLoop
        .pid(PID_P, PID_I, PID_D)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .allowedClosedLoopError(0.5, ClosedLoopSlot.kSlot0);

    extendMotor.configure(
        extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = extendMotor.getEncoder();
    controller = extendMotor.getClosedLoopController();
    limitSwitch = new DigitalInput(EXTEND_LIMIT_ID);
  }

  @Override
  public void updateInputs(IntakeExtendIOInputs inputs) {
    var position = encoder.getPosition();
    var velocity = encoder.getVelocity();
    var appliedOutput = extendMotor.getAppliedOutput();
    var busVoltage = extendMotor.getBusVoltage();
    var current = extendMotor.getOutputCurrent();
    var temperature = extendMotor.getMotorTemperature();
    inputs.connected =
        position.isValid()
            && velocity.isValid()
            && appliedOutput.isValid()
            && busVoltage.isValid()
            && current.isValid();
    inputs.position = position.get();
    inputs.velocity = velocity.get() / 60.0;
    inputs.appliedVolts = appliedOutput.get() * busVoltage.get();
    inputs.outputCurrentAmps = current.get();
    inputs.temperatureCelsius = temperature.get();
    inputs.limitSwitchPressed = limitSwitch.get();
  }

  @Override
  public void applyOutputs(IntakeExtendIOOutputs outputs) {
    switch (outputs.mode) {
      case POSITION ->
          controller.setSetpoint(outputs.targetPosition, SparkLowLevel.ControlType.kPosition);
      case VOLTAGE -> extendMotor.setVoltage(outputs.targetVolts);
      case STOP -> extendMotor.stopMotor();
    }
  }

  @Override
  public void resetPosition() {
    encoder.setPosition(0.0);
  }
}
