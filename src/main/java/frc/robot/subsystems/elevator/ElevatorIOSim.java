package frc.robot.subsystems.elevator;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
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

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          neo,
          Constants.ELEVATOR_GEARING,
          Constants.CARRIAGE_MASS_KG,
          Constants.MIN_ELEVATOR_HEIGHT_METERS,
          Constants.MIN_ELEVATOR_HEIGHT_METERS,
          Constants.MAX_ELEVATOR_HEIGHT_METERS,
          Constants.SIMULATE_GRAVITY,
          Constants.STARTING_HEIGHT_METERS,
          Constants.MEASUREMENT_STD_DEVS,
          0.0);

  private final SparkMax motor = new SparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxSim motorSim = new SparkMaxSim(motor, neo);
  private final SparkAbsoluteEncoderSim encoderSim = motorSim.getAbsoluteEncoderSim();

  // private final EncoderSim encoderSim = motor.getAbsoluteEncoderSim();

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d elevatorMech2d =
      mech2dRoot.append(new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

  public ElevatorIOSim() {

    // Set the distance per pulse for the encoder
    encoderSim.setPosition(0);
    encoderSim.setVelocity(0);
    encoderSim.setPositionConversionFactor(Constants.ELEVATOR_ENCODER_DISTANCE_PER_PULSE);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", mech2d);
  }

  @Override
  public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {

    elevatorSim.setInput(motorSim.getVelocity());

    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    encoderSim.setPosition(elevatorSim.getPositionMeters());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    elevatorMech2d.setLength(encoderSim.getPosition());

    inputs.amps = motorSim.getMotorCurrent();
    inputs.velocity = motorSim.getVelocity();
    inputs.position = motorSim.getPosition();
    inputs.setpoint = motorSim.getSetpoint();
    inputs.volts = motorSim.getAppliedOutput();
  }

  @Override
  public void setVoltage(double volts) {
    motorSim.setBusVoltage(volts);
  }
}
