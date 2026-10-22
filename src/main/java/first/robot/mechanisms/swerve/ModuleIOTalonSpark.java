package first.robot.mechanisms.swerve;

import static first.robot.Schematic.*;
import static first.robot.mechanisms.swerve.DriveConstants.*;
import static first.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.Queue;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class ModuleIOTalonSpark implements ModuleIO {
  private final Rotation2d zeroRotation;

  private final TalonFX driveTalon;
  private final SparkMax turnSpark;
  private final CANcoder cancoder;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  private final PIDController turnController = new PIDController(turnKp, 0.0, turnKd);
  private Rotation2d turnPosition = Rotation2d.ZERO;
  private boolean turnClosedLoop = false;

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<AngularVelocity> turnVelocity;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.RISING);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.RISING);

  public ModuleIOTalonSpark(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> Rotation2d.ZERO;
        };
    driveTalon =
        new TalonFX(
            switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            },
            canBus);
    turnSpark =
        new SparkMax(
            canPort,
            switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    cancoder =
        new CANcoder(
            switch (module) {
              case 0 -> frontLeftAbsoluteEncoderCanId;
              case 1 -> frontRightAbsoluteEncoderCanId;
              case 2 -> backLeftAbsoluteEncoderCanId;
              case 3 -> backRightAbsoluteEncoderCanId;
              default -> 0;
            },
            canBus);

    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    driveConfig.Feedback.SensorToMechanismRatio = driveMotorReduction;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = driveMotorCurrentLimit;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -driveMotorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimit = driveMotorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    var slot0config = new Slot0Configs();
    slot0config.kP = driveKp;
    slot0config.kI = 0;
    slot0config.kD = driveKd;
    slot0config.kS = driveKs;
    slot0config.kV = driveKv;
    slot0config.kA = driveKa;
    slot0config.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    driveConfig.withSlot0(slot0config);
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = 0.0;
    cancoderConfig.MagnetSensor.SensorDirection =
        turnEncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnVelocity = cancoder.getVelocity();

    timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = OdometryThread.getInstance().registerSignal(drivePosition);
    turnPositionQueue = OdometryThread.getInstance().registerSignal(turnAbsolutePosition);

    BaseStatusSignal.setUpdateFrequencyForAll(
        odometryFrequency, drivePosition, turnAbsolutePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, driveVelocity, driveAppliedVolts, driveCurrent, turnVelocity);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, cancoder);

    turnController.enableContinuousInput(turnPIDMinInput, turnPIDMaxInput);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    var turnStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition, turnVelocity);

    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    var appliedOutput = turnSpark.getAppliedOutput();
    var busVoltage = turnSpark.getBusVoltage();
    var outputCurrent = turnSpark.getOutputCurrent();
    boolean sparkOk = appliedOutput.isValid() && busVoltage.isValid() && outputCurrent.isValid();
    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK() && sparkOk);
    inputs.turnPositionAbsolute =
        Units.rotationsToRadians(turnAbsolutePosition.getValueAsDouble());
    turnPosition = new Rotation2d(inputs.turnPositionAbsolute).minus(zeroRotation);
    inputs.turnPosition = turnPosition;
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    if (sparkOk) {
      inputs.turnAppliedVolts = appliedOutput.get() * busVoltage.get();
      inputs.turnCurrentAmps = outputCurrent.get();
    }

    if (turnClosedLoop) {
      runTurnPID();
    }

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (driveClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveTalon.setControl(
        switch (driveClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
    runTurnPID();
  }

  private void runTurnPID() {
    double volts = turnController.calculate(turnPosition.getRadians());
    turnSpark.setVoltage(Math.clamp(volts, -12.0, 12.0));
  }
}
