package frc.robot.subsystems.coral;

import static frc.robot.Schematic.coralMotorID;
import static frc.robot.Schematic.hopperReflectorSensorDioId;
import static frc.robot.subsystems.coral.CoralEffectorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralEffectorIOSparkMAX implements CoralEffectorIO {

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  // private final RelativeEncoder effectorEncoder;
  private final DigitalInput photosensor;

  public CoralEffectorIOSparkMAX() {
    motor = new SparkMax(coralMotorID, SparkLowLevel.MotorType.kBrushless);
    encoder = motor.getEncoder();
    photosensor = new DigitalInput(hopperReflectorSensorDioId);

    SparkMaxConfig effectorConfig = new SparkMaxConfig();
    effectorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(effectorCurrentMotorLimit)
        .voltageCompensation(12.0);

    effectorConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
    effectorConfig
        .encoder
        .positionConversionFactor(effectorEncoderPositionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
  }

  public void updateInputs(CoralEffectorIOInputs inputs) {
    inputs.appliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.temperature = motor.getMotorTemperature();
    inputs.velocity = motor.getAbsoluteEncoder().getVelocity();
    inputs.hopperSeesCoral = photosensor.get();
    inputs.hasCoral = motor.getForwardLimitSwitch().isPressed();
  }

  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    System.out.println("Setting Speed: " + speed);
    motor.set(speed);
  }
}
