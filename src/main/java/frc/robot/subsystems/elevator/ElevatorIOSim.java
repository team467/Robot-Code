package frc.robot.subsystems.elevator;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorIOSim implements ElevatorIO {

  private final DCMotor neo =
      DCMotor.getNEO(Constants.ELEVATOR_NUM_MOTORS).withReduction(Constants.ELEVATOR_GEAR_RATIO);
  private final SparkMax motor = new SparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxSim motorSim = new SparkMaxSim(motor, neo);
  private final SparkRelativeEncoder encoder = (SparkRelativeEncoder) motor.getEncoder();
  private final SparkRelativeEncoderSim encoderSim = new SparkRelativeEncoderSim(motor);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          neo,
          Constants.ELEVATOR_GEARING,
          Constants.CARRIAGE_MASS_KG,
          Constants.ELEVATOR_DRUM_RADIUS,
          Constants.MIN_ELEVATOR_HEIGHT_METERS,
          Constants.MAX_ELEVATOR_HEIGHT_METERS,
          Constants.SIMULATE_GRAVITY,
          Constants.STARTING_HEIGHT_METERS,
          Constants.MEASUREMENT_STD_DEVS,
          0.0);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d elevatorMech2d =
      mech2dRoot.append(new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

  public ElevatorIOSim() {

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    motorConfig.inverted(false);
    motorConfig.smartCurrentLimit(80, 80, 5700);
    motorConfig.voltageCompensation(12.0);

    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(Constants.ELEVATOR_ENCODER_DISTANCE_PER_PULSE);
    encoderConfig.velocityConversionFactor(Constants.ELEVATOR_ENCODER_DISTANCE_PER_PULSE / 60.0);
    motorConfig.apply(encoderConfig);

    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Set the distance per pulse for the encoder
    encoder.setPosition(0.0);
    motorSim.enable();
    motorSim.setPosition(0.0);
    encoderSim.setPosition(0.0);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", mech2d);
  }

  @Override
  public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {

    // First we simulate the motor to updates its state including voltage output
    motorSim.iterate(motor.get(), RobotController.getBatteryVoltage(), 0.020);

    // Next, we set the elevatorSim's input to the motor's output. Note that applied output needs to
    // by multiplied by bus voltage.
    elevatorSim.setInput(motor.getAppliedOutput() * motor.getBusVoltage());

    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    encoderSim.setPosition(elevatorSim.getPositionMeters());
    encoderSim.setVelocity(elevatorSim.getVelocityMetersPerSecond());

    // SimBattery estimates loaded battery voltages across all simulations
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    elevatorMech2d.setLength(encoderSim.getPosition());

    // Update the inputs
    inputs.amps = elevatorSim.getCurrentDrawAmps();
    inputs.velocity = encoderSim.getVelocity();
    inputs.position = encoderSim.getPosition();
    inputs.setpoint = motor.get();
    inputs.volts = motorSim.getAppliedOutput() * motorSim.getBusVoltage();
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
