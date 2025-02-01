package frc.robot.subsystems.coral;

import static frc.lib.utils.SparkUtil.tryUntilOk;
import static frc.robot.Schematic.coralMotorID;
import static frc.robot.Schematic.hopperReflectorSensorDioId;
import static frc.robot.subsystems.coral.CoralEffectorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralEffectorIOSparkMAX implements CoralEffectorIO {

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final DigitalInput photosensor;

  public CoralEffectorIOSparkMAX() {
    motor = new SparkMax(coralMotorID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    photosensor = new DigitalInput(hopperReflectorSensorDioId);

    SparkMaxConfig effectorConfig = new SparkMaxConfig();
    effectorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(effectorCurrentMotorLimit)
        .voltageCompensation(12.0);

    effectorConfig.limitSwitch.forwardLimitSwitchEnabled(false);

    effectorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);

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
                effectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }
}
