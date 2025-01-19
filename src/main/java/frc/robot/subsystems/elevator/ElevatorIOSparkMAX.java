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

public class ElevatorIOSparkMAX implements ElevatorIO {
  private SparkMax Spark;
  private RelativeEncoder Encoder;
  private final SparkClosedLoopController Controller;

  public ElevatorIOSparkMAX(int elevator) {
    Spark = new SparkMax(elevatorCanId, MotorType.kBrushless);
    var Config = new SparkMaxConfig();
    Config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(elevatorCurrentLimit);
    tryUntilOk(
        Spark,
        5,
        () ->
            Spark.configure(
                Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(Spark, 5, () -> Encoder.setPosition(0.0));
    Controller = Spark.getClosedLoopController();
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.velocityMetersPerSec = (Encoder.getVelocity() * 2 * Math.PI) * 0.004;
  }
}
