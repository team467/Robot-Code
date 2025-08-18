package frc.robot.subsystems.elevator;

import static frc.lib.utils.SparkUtil.tryUntilOk;
import static frc.robot.subsystems.elevator.ElevatorConstants.ENCODER_CONVERSION_FACTOR;
import static frc.robot.subsystems.elevator.ElevatorConstants.elevatorCurrentLimit;
import static frc.robot.subsystems.elevator.ElevatorConstants.elevatorToGround;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIOSim implements ElevatorIO {

  private final DCMotor neo = DCMotor.getNEO(1).withReduction(1);
  private final SparkMax motor = new SparkMax(3, MotorType.kBrushless);
  private final SparkMaxSim motorSim = new SparkMaxSim(motor, neo);
  private final SparkRelativeEncoder encoder = (SparkRelativeEncoder) motor.getEncoder();
  private final SparkRelativeEncoderSim encoderSim = new SparkRelativeEncoderSim(motor);
  private boolean isCalibrated = false;
  private final SparkClosedLoopController controller = motor.getClosedLoopController();

  private double setpoint;

  public ElevatorIOSim() {
    var config = new SparkFlexConfig();
    config
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(elevatorCurrentLimit)
        .voltageCompensation(12.0);
    config
        .externalEncoder
        .positionConversionFactor(ENCODER_CONVERSION_FACTOR)
        .velocityConversionFactor(ENCODER_CONVERSION_FACTOR / 60)
        .inverted(true)
        .averageDepth(2);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .positionWrappingEnabled(false)
        .pidf(15, 0.0, 7, 0.0); // p:4.6 d: 24
    config
        .signals
        .externalOrAltEncoderPositionAlwaysOn(true)
        .externalOrAltEncoderPosition(20)
        .externalOrAltEncoderVelocityAlwaysOn(true)
        .externalOrAltEncoderVelocity(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    config
        .softLimit
        .forwardSoftLimit(0.79)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(false);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(motor, 5, () -> encoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    motorSim.iterate(motor.get(), RobotController.getBatteryVoltage(), 0.020);
    inputs.positionMeters = motorSim.getPosition();
    inputs.velocityMetersPerSec = motorSim.getVelocity();
    inputs.appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = motorSim.getMotorCurrent();
    inputs.setpoint = setpoint;
    inputs.stowLimitSwitch = !(inputs.positionMeters == 0.42);
    inputs.atSetpoint = Math.abs(setpoint - inputs.positionMeters) < ElevatorConstants.TOLERANCE;
    if (inputs.stowLimitSwitch) {
      this.isCalibrated = true;
      motorSim.setPosition(elevatorToGround);
    }
    inputs.isCalibrated = this.isCalibrated;
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }

  @Override
  public void setPosition(double setpoint) {
    this.setpoint = setpoint;
  }

  @Override
  public void goToSetpoint() {
    if (!isCalibrated) {
      setPercent(-0.15);
    } else {
      controller.setReference(this.setpoint, SparkBase.ControlType.kPosition);
    }
  }
}
