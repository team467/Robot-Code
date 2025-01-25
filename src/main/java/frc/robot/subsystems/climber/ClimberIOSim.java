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

  private final DCMotor neo =
      DCMotor.getNEO(ClimberConstants.CLIMBER_NUM_MOTORS)
          .withReduction(ClimberConstants.CLIMBER_GEAR_RATIO);
  private final SparkMax motor =
      new SparkMax(ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxSim motorSim = new SparkMaxSim(motor, neo);
  private final SparkRelativeEncoder encoder = (SparkRelativeEncoder) motor.getEncoder();
  private final SparkRelativeEncoderSim encoderSim = new SparkRelativeEncoderSim(motor);

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

    encoder.setPosition(0.0);
    motorSim.enable();
    motorSim.setPosition(0.0);
    encoderSim.setPosition(0.0);

    SmartDashboard.putData("Climber Sim", mech2d);
  }

  @Override
  public void updateInputs(ClimberIO.ClimberIOInputs inputs) {
    motorSim.iterate(motor.get(), RobotController.getBatteryVoltage(), 0.020);

    elevatorSim.setInput(motor.getAppliedOutput() * motor.getBusVoltage());

    elevatorSim.update(0.020);

    encoderSim.setPosition(elevatorSim.getPositionMeters());
    encoderSim.setVelocity(elevatorSim.getVelocityMetersPerSecond());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    elevatorMech2d.setLength(encoderSim.getPosition());

    inputs.current = elevatorSim.getCurrentDrawAmps();
    inputs.speed = encoderSim.getVelocity();
    inputs.position = encoderSim.getPosition();
    inputs.volts = motorSim.getAppliedOutput() * motorSim.getBusVoltage();
    inputs.climberWinched = inputs.position <= ClimberConstants.WINCHED_POSITION;
    inputs.climberDeployed = inputs.position >= ClimberConstants.MAX_CLIMBER_HEIGHT_METERS;
    inputs.climberStowed = inputs.position <= 0.0;

    // Reset position if the stowed limit switch is pressed
    if (inputs.climberStowed) {
      resetPosition();
    }

    // Handle GUI inputs
    if (SmartDashboard.getBoolean("Climber/Deploy", false)) {
      motor.set(1.0); // Extend at full speed
      if (inputs.position >= ClimberConstants.MAX_CLIMBER_HEIGHT_METERS) {
        motor.set(0.0); // Stop the motor
        SmartDashboard.putBoolean("Climber/Deploy", false);
      }
    }

    if (SmartDashboard.getBoolean("Climber/Winch", false)) {
      motor.set(-1.0); // Retract at full speed
      if (inputs.position <= ClimberConstants.WINCHED_POSITION) {
        motor.set(0.0); // Stop the motor
        SmartDashboard.putBoolean("Climber/Winch", false);
      }
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
