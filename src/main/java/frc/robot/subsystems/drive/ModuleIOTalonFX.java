package frc.robot.subsystems.drive;

import static frc.lib.utils.PhoenixUtil.*;
import static frc.robot.Schematic.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;

/**
 * Module IO implementation for TalonFX drive motor controller, TalonFX turn motor controller,
 * and CANcoder absolute encoder.
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder turnEncoderAbsolute;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  // Input from turn encoder absolute
  private final StatusSignal<Angle> turnPositionAbsolute;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

  public ModuleIOTalonFX(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> Rotation2d.kZero;
        };
    driveTalon =
        new TalonFX(
            switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            });
    turnTalon =
        new TalonFX(
            switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            });
    turnEncoderAbsolute =
        new CANcoder(
            switch (module) {
              case 0 -> frontLeftAbsoluteEncoderCanId;
              case 1 -> frontRightAbsoluteEncoderCanId;
              case 2 -> backLeftAbsoluteEncoderCanId;
              case 3 -> backRightAbsoluteEncoderCanId;
              default -> 0;
            });

    // Configure drive motor
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0.kP = driveKp;
    driveConfig.Slot0.kI = 0;
    driveConfig.Slot0.kD = driveKd;
    driveConfig.Slot0.kS = driveKs;
    driveConfig.Slot0.kV = driveKv;
    driveConfig.Slot0.kA = driveKa;
    driveConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    driveConfig.Feedback.SensorToMechanismRatio = driveMotorReduction;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = driveMotorCurrentLimit;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -driveMotorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimit = driveMotorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    var turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0.kP = turnKp;
    turnConfig.Slot0.kI = 0;
    turnConfig.Slot0.kD = turnKd;
    turnConfig.Feedback.FeedbackRemoteSensorID = turnEncoderAbsolute.getDeviceID();
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.RotorToSensorRatio = turnMotorReduction;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / turnMotorReduction;
    turnConfig.MotionMagic.MotionMagicAcceleration = turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * turnMotorReduction;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = turnMotorCurrentLimit;
    turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -turnMotorCurrentLimit;
    turnConfig.CurrentLimits.StatorCurrentLimit = turnMotorCurrentLimit;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.MotorOutput.Inverted = turnInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = zeroRotation.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection =
        turnEncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    turnEncoderAbsolute.getConfigurator().apply(cancoderConfig);

    // Create timestamp queue
    timestampQueue = OdometryThread.getInstance().makeTimestampQueue();

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue = OdometryThread.getInstance().registerSignal(driveTalon.getPosition().clone());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    // Create turn status signals
    turnPosition = turnTalon.getPosition();
    turnPositionQueue = OdometryThread.getInstance().registerSignal(turnTalon.getPosition().clone());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();
    turnPositionAbsolute = turnEncoderAbsolute.getPosition();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(odometryFrequency, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, driveVelocity, driveAppliedVolts, driveCurrent, 
        turnVelocity, turnAppliedVolts, turnCurrent, turnPositionAbsolute);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon, turnEncoderAbsolute);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh all signals
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    var turnStatus =
        BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent, turnPositionAbsolute);

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Update turn inputs
    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
    inputs.turnPositionAbsolute = Units.rotationsToRadians(turnPositionAbsolute.getValueAsDouble());

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
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
    turnTalon.setControl(voltageRequest.withOutput(output));
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
    turnTalon.setControl(positionVoltageRequest.withPosition(rotation.getRotations()));
  }
}