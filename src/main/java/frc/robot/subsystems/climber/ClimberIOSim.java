package frc.robot.subsystems.climber;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberIOSim implements ClimberIO {

  private final DCMotor neo = DCMotor.getNEO(2).withReduction(ClimberConstants.CLIMBER_GEAR_RATIO);
  private final SparkMax motor = new SparkMax(1, MotorType.kBrushless);
  private final SparkMaxSim motorSim = new SparkMaxSim(motor, neo);
  private final SparkRelativeEncoder encoder = (SparkRelativeEncoder) motor.getEncoder();
  private final SparkRelativeEncoderSim encoderSim = new SparkRelativeEncoderSim(motor);

  // Simulation physics
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          neo,
          ClimberConstants.CLIMBER_GEARING,
          ClimberConstants.CARRIAGE_MASS_KG,
          ClimberConstants.CLIMBER_DRUM_RADIUS,
          ClimberConstants.MIN_CLIMBER_HEIGHT_METERS,
          ClimberConstants.MAX_CLIMBER_HEIGHT_METERS,
          ClimberConstants.SIMULATE_GRAVITY,
          ClimberConstants.STARTING_HEIGHT_METERS,
          ClimberConstants.MEASUREMENT_STD_DEVS,
          0.0);

  // Visualization
  private final Mechanism2d mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Climber Root", 10, 0);
  private final MechanismLigament2d elevatorMech2d =
      mech2dRoot.append(new MechanismLigament2d("Climber", elevatorSim.getPositionMeters(), 90));

  public ClimberIOSim() {
    var motorConfig = new SparkFlexConfig();
    motorConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(40);
    motorConfig
        .softLimit
        .forwardSoftLimit(10)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(false);

    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(ClimberConstants.CLIMBER_CONVERSION_FACTOR);
    encoderConfig.velocityConversionFactor(ClimberConstants.CLIMBER_CONVERSION_FACTOR / 60.0);
    motorConfig.apply(encoderConfig);

    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Reset encoders and simulation state
    encoder.setPosition(0.0);
    motorSim.enable();
    motorSim.setPosition(0.0);
    encoderSim.setPosition(0.0);

    // Publish Mechanism2d to SmartDashboard
    SmartDashboard.putData("Climber Sim", mech2d);
  }

  @Override
  public void updateInputs(ClimberIO.ClimberIOInputs inputs) {
    // Simulate motor state, including voltage output
    motorSim.iterate(motor.get(), RobotController.getBatteryVoltage(), 0.020);

    // Set the elevatorSim's input to the motor's output
    elevatorSim.setInput(motor.getAppliedOutput() * motor.getBusVoltage());

    // Update elevatorSim
    elevatorSim.update(0.020);

    // Set simulated encoder readings and battery voltage
    encoderSim.setPosition(elevatorSim.getPositionMeters());
    encoderSim.setVelocity(elevatorSim.getVelocityMetersPerSecond());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    elevatorMech2d.setLength(encoderSim.getPosition());

    // Update inputs
    inputs.current = elevatorSim.getCurrentDrawAmps();
    inputs.speed = encoderSim.getVelocity();
    inputs.position = encoderSim.getPosition();
    inputs.volts = motorSim.getAppliedOutput() * motorSim.getBusVoltage();
    inputs.climberWinched =
        inputs.position >= ClimberConstants.LOWER_WINCHED_POSITION
            && inputs.position <= ClimberConstants.UPPER_WINCHED_POSITION;
    inputs.climberDeployed =
        inputs.position >= ClimberConstants.LOWER_DEPLOYED_POSITION
            && inputs.position <= ClimberConstants.UPPER_DEPLOYED_POSITION;

    // Reset position if the stowed limit switch is pressed
    if (inputs.climberDeployed) {
      resetPosition();
    }
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }
}
