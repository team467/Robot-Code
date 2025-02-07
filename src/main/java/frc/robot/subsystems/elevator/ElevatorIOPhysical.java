package frc.robot.subsystems.elevator;

import static frc.lib.utils.SparkUtil.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.io.ObjectInputFilter.Config;

public class ElevatorIOPhysical implements ElevatorIO {
  private final SparkMax Spark;
  private final RelativeEncoder Encoder;
  private final SparkClosedLoopController Controller;
  private final SparkLimitSwitch elevatorStowLimitSwitch;

  public ElevatorIOPhysical() {
    Spark = new SparkMax(elevatorCanId, MotorType.kBrushless);
    Encoder = Spark.getEncoder();
    elevatorStowLimitSwitch = Spark.getReverseLimitSwitch();
    Controller = Spark.getClosedLoopController();

    var Config = new SparkMaxConfig();
    Config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(elevatorCurrentLimit);
    tryUntilOk(
        Spark,
        5,
        () ->
            Spark.configure(
                Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(Spark, 5, () -> Encoder.setPosition(0.0));
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.velocityMetersPerSec = (Encoder.getVelocity() * 2 * Math.PI) * 0.004;
    inputs.elevatorAppliedVolts = Spark.getBusVoltage() * Spark.getAppliedOutput();
    inputs.elevatorCurrentAmps = Spark.getOutputCurrent();
    inputs.positionMeters = Encoder.getPosition();
    inputs.limitSwitchPressed = elevatorStowLimitSwitch.isPressed();
  }

  @Override
  public void setPercent(double percent) {
    Spark.set(percent);
  }

  @Override
  public void setVoltage(double volts) {
    Spark.setVoltage(volts);
  }
}
