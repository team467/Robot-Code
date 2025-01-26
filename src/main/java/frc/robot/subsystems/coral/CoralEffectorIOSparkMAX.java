package frc.robot.subsystems.coral;

import static frc.lib.utils.SparkUtil.tryUntilOk;
import static frc.robot.Schematic.coralHaveCoralDioId;
import static frc.robot.Schematic.coralMotorID;
import static frc.robot.Schematic.coralOnTheWayDioId;
import static frc.robot.subsystems.coral.CoralEffectorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralEffectorIOSparkMAX implements CoralEffectorIO {

  private final SparkMax motor;
  private final RelativeEncoder effectorEncoder;
  private final DigitalInput effectorLimitSwitchHaveCoral = new DigitalInput(coralHaveCoralDioId);
  private final DigitalInput photosensor;

  public CoralEffectorIOSparkMAX() {
    motor = new SparkMax(coralMotorID, SparkLowLevel.MotorType.kBrushless);
    effectorEncoder = motor.getEncoder();
    photosensor = new DigitalInput(coralOnTheWayDioId);

    // effectorMotor.configure();

    var effectorConfig = new SparkFlexConfig();
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

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                effectorConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    tryUntilOk(motor, 5, () -> effectorEncoder.setPosition(0.0));
  }

  public void updateInputs(CoralEffectorIOInputs inputs) {
    inputs.appliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.temperature = motor.getMotorTemperature();
    inputs.velocity = motor.getAbsoluteEncoder().getVelocity();
    inputs.coralOnTheWay = photosensor.get();
    inputs.haveCoral = effectorLimitSwitchHaveCoral.get();

  }

  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }
}

// Rename Coral, add encoder, find things to add for motor.configure(); such as motorid,
// temeprature, etc.

// Check MotorIOConfig file to get important implementations to code
// Three main things to add found in Drive something
