package first.robot.mechanisms.shooter;

import static first.robot.Schematic.shooterBottomLeftMotorCanId;
import static first.robot.Schematic.shooterBottomRightMotorCanId;
import static first.robot.Schematic.shooterTopLeftMotorCanId;
import static first.robot.Schematic.shooterTopRightMotorCanId;
import static first.robot.mechanisms.shooter.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIOSparkMax implements ShooterIO {
  private final SparkMax topLeftMotor;
  private final SparkMax topRightMotor;
  private final SparkMax bottomLeftMotor;
  private final SparkMax bottomRightMotor;
  private final RelativeEncoder topLeftMotorEncoder;
  private final RelativeEncoder topRightMotorEncoder;
  private final RelativeEncoder bottomLeftMotorEncoder;
  private final RelativeEncoder bottomRightMotorEncoder;

  public ShooterIOSparkMax() {
    topLeftMotor = new SparkMax(CAN_PORT, shooterTopLeftMotorCanId, MotorType.kBrushless);
    topRightMotor = new SparkMax(CAN_PORT, shooterTopRightMotorCanId, MotorType.kBrushless);
    bottomLeftMotor = new SparkMax(CAN_PORT, shooterBottomLeftMotorCanId, MotorType.kBrushless);
    bottomRightMotor = new SparkMax(CAN_PORT, shooterBottomRightMotorCanId, MotorType.kBrushless);

    bottomLeftMotor.configure(
        motorConfig(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topLeftMotor.configure(
        motorConfig(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomRightMotor.configure(
        motorConfig(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topRightMotor.configure(
        motorConfig(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    topLeftMotorEncoder = topLeftMotor.getEncoder();
    topRightMotorEncoder = topRightMotor.getEncoder();
    bottomLeftMotorEncoder = bottomLeftMotor.getEncoder();
    bottomRightMotorEncoder = bottomRightMotor.getEncoder();
  }

  private static SparkMaxConfig motorConfig(boolean inverted) {
    var config = new SparkMaxConfig();
    config
        .inverted(inverted)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(CURRENT_LIMIT)
        .secondaryCurrentLimit(CURRENT_LIMIT);
    return config;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    boolean connected = true;

    var tlCurrent = topLeftMotor.getOutputCurrent();
    var tlBus = topLeftMotor.getBusVoltage();
    var tlOutput = topLeftMotor.getAppliedOutput();
    var tlVelocity = topLeftMotorEncoder.getVelocity();
    connected &= tlCurrent.isValid() && tlBus.isValid() && tlOutput.isValid() && tlVelocity.isValid();
    inputs.topLeftMotorCurrentAmps = tlCurrent.get();
    inputs.topLeftMotorAppliedVolts = tlBus.get() * tlOutput.get();
    inputs.topLeftMotorVelocityRadPerSec = tlVelocity.get() * ENCODER_VELOCITY_CONVERSION;

    var trCurrent = topRightMotor.getOutputCurrent();
    var trBus = topRightMotor.getBusVoltage();
    var trOutput = topRightMotor.getAppliedOutput();
    var trVelocity = topRightMotorEncoder.getVelocity();
    var trPosition = topRightMotorEncoder.getPosition();
    connected &=
        trCurrent.isValid()
            && trBus.isValid()
            && trOutput.isValid()
            && trVelocity.isValid()
            && trPosition.isValid();
    inputs.topRightMotorCurrentAmps = trCurrent.get();
    inputs.topRightMotorAppliedVolts = trBus.get() * trOutput.get();
    inputs.busVoltage = trBus.get();
    inputs.topRightMotorVelocityRadPerSec = trVelocity.get() * ENCODER_VELOCITY_CONVERSION;

    var blCurrent = bottomLeftMotor.getOutputCurrent();
    var blBus = bottomLeftMotor.getBusVoltage();
    var blOutput = bottomLeftMotor.getAppliedOutput();
    var blVelocity = bottomLeftMotorEncoder.getVelocity();
    connected &= blCurrent.isValid() && blBus.isValid() && blOutput.isValid() && blVelocity.isValid();
    inputs.bottomLeftMotorCurrentAmps = blCurrent.get();
    inputs.bottomLeftMotorAppliedVolts = blBus.get() * blOutput.get();
    inputs.bottomLeftMotorVelocityRadPerSec = blVelocity.get() * ENCODER_VELOCITY_CONVERSION;

    var brCurrent = bottomRightMotor.getOutputCurrent();
    var brBus = bottomRightMotor.getBusVoltage();
    var brOutput = bottomRightMotor.getAppliedOutput();
    var brVelocity = bottomRightMotorEncoder.getVelocity();
    connected &= brCurrent.isValid() && brBus.isValid() && brOutput.isValid() && brVelocity.isValid();
    inputs.bottomRightMotorCurrentAmps = brCurrent.get();
    inputs.bottomRightMotorAppliedVolts = brBus.get() * brOutput.get();
    inputs.bottomRightMotorVelocityRadPerSec = brVelocity.get() * ENCODER_VELOCITY_CONVERSION;

    inputs.connected = connected;
    inputs.totalAmps =
        inputs.topLeftMotorCurrentAmps
            + inputs.topRightMotorCurrentAmps
            + inputs.bottomLeftMotorCurrentAmps
            + inputs.bottomRightMotorCurrentAmps;

    inputs.shooterWheelVelocityRadPerSec =
        inputs.topRightMotorVelocityRadPerSec / SHOOTER_WHEEL_GEAR_RATIO;
    inputs.shooterWheelPosition =
        trPosition.get() * ENCODER_POSITION_CONVERSION / SHOOTER_WHEEL_GEAR_RATIO;
  }

  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    switch (outputs.mode) {
      case VOLTAGE -> {
        topRightMotor.setVoltage(outputs.targetVolts);
        topLeftMotor.setVoltage(outputs.targetVolts);
        bottomRightMotor.setVoltage(outputs.targetVolts);
        bottomLeftMotor.setVoltage(outputs.targetVolts);
      }
      case STOP -> {
        topLeftMotor.stopMotor();
        topRightMotor.stopMotor();
        bottomLeftMotor.stopMotor();
        bottomRightMotor.stopMotor();
      }
    }
  }
}
