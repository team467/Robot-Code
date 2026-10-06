package frc.robot.subsystems.drive;

import static frc.lib.utils.PhoenixUtil.*;
import static frc.robot.Schematic.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;


/**
 * Module IO implementation for Talon FX drive motor controller,
 * Talon FX turn motor controller, and CANcoder absolute encoder.
 */
public class ModuleIOTalonSpark implements ModuleIO {

  // =========================================================================
  // Phoenix odometry thread
  // =========================================================================
  //
  // This is implemented here instead of using AdvantageKit's
  // PhoenixOdometryThread because that class is not present in this project.
  //
  // Phoenix signals are updated at the configured odometry frequency and
  // copied into queues. The normal ModuleIO updateInputs() method consumes
  // those queues.
  //
  // =========================================================================

  private static final class PhoenixOdometryThread extends Thread {

    private static final PhoenixOdometryThread INSTANCE =
        new PhoenixOdometryThread();

    private final List<BaseStatusSignal> signals = new ArrayList<>();
    private final List<Queue<Double>> signalQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private PhoenixOdometryThread() {
      super("PhoenixOdometryThread");

      setDaemon(true);
      start();
    }

    public static PhoenixOdometryThread getInstance() {
      return INSTANCE;
    }

    public synchronized Queue<Double> makeTimestampQueue() {
      Queue<Double> queue = new ConcurrentLinkedQueue<>();

      timestampQueues.add(queue);

      return queue;
    }

    public synchronized Queue<Double> registerSignal(
        StatusSignal<Angle> signal) {

      Queue<Double> queue = new ConcurrentLinkedQueue<>();

      signals.add(signal);
      signalQueues.add(queue);

      return queue;
    }

    @Override
    public void run() {

      while (!Thread.currentThread().isInterrupted()) {

        BaseStatusSignal[] signalsSnapshot;

        synchronized (this) {
          signalsSnapshot =
              signals.toArray(new BaseStatusSignal[0]);
        }

        if (signalsSnapshot.length == 0) {
          try {
            Thread.sleep(1);
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            break;
          }

          continue;
        }

        /*
         * Wait until a new sample is available from all registered
         * Phoenix signals.
         *
         * This is preferable to using Timer.getFPGATimestamp() because
         * Phoenix provides timestamps associated with the received CAN data.
         */
        BaseStatusSignal.waitForAll(
            0.1,
            signalsSnapshot);

        /*
         * Use the timestamp from the first Phoenix signal.
         *
         * All odometry signals are configured at the same frequency and
         * are sampled together by waitForAll().
         */
        double timestamp =
            signalsSnapshot[0]
                .getTimestamp()
                .getTime();

        synchronized (this) {

          /*
           * Every ModuleIO instance gets its own timestamp queue.
           */
          for (Queue<Double> queue : timestampQueues) {
            queue.add(timestamp);
          }

          /*
           * Copy every registered signal into its corresponding queue.
           */
          for (int i = 0; i < signals.size(); i++) {

            signalQueues
                .get(i)
                .add(
                    signals
                        .get(i)
                        .getValueAsDouble());
          }
        }
      }
    }
  }


  // =========================================================================
  // Module state
  // =========================================================================

  private final Rotation2d zeroRotation;


  // =========================================================================
  // Hardware objects
  // =========================================================================

  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder turnEncoderAbsolute;


  // =========================================================================
  // Voltage control requests
  // =========================================================================

  private final VoltageOut voltageRequest =
      new VoltageOut(0);

  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0);

  private final VelocityVoltage velocityVoltageRequest =
      new VelocityVoltage(0.0);


  // =========================================================================
  // Torque-current control requests
  // =========================================================================

  private final TorqueCurrentFOC torqueCurrentRequest =
      new TorqueCurrentFOC(0);

  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);

  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);


  // =========================================================================
  // Timestamp inputs from Phoenix odometry thread
  // =========================================================================

  private final Queue<Double> timestampQueue;


  // =========================================================================
  // Drive motor inputs
  // =========================================================================

  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;

  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;


  // =========================================================================
  // Turn motor inputs
  // =========================================================================

  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;

  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;


  // =========================================================================
  // Absolute CANcoder input
  // =========================================================================

  private final StatusSignal<Angle> turnAbsolutePosition;


  // =========================================================================
  // Connection debouncers
  // =========================================================================

  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5);

  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5);

  private final Debouncer turnEncoderConnectedDebounce =
      new Debouncer(0.5);


  // =========================================================================
  // Constructor
  // =========================================================================

  public ModuleIOTalonSpark(int module) {

    // -----------------------------------------------------------------------
    // Module-specific zero rotation
    // -----------------------------------------------------------------------

    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> Rotation2d.ZERO;
        };


    // -----------------------------------------------------------------------
    // CAN IDs
    // -----------------------------------------------------------------------

    int driveCanId =
        switch (module) {
          case 0 -> frontLeftDriveCanId;
          case 1 -> frontRightDriveCanId;
          case 2 -> backLeftDriveCanId;
          case 3 -> backRightDriveCanId;
          default -> 0;
        };

    int turnCanId =
        switch (module) {
          case 0 -> frontLeftTurnCanId;
          case 1 -> frontRightTurnCanId;
          case 2 -> backLeftTurnCanId;
          case 3 -> backRightTurnCanId;
          default -> 0;
        };

    int encoderCanId =
        switch (module) {
          case 0 -> frontLeftAbsoluteEncoderCanId;
          case 1 -> frontRightAbsoluteEncoderCanId;
          case 2 -> backLeftAbsoluteEncoderCanId;
          case 3 -> backRightAbsoluteEncoderCanId;
          default -> 0;
        };


    // -----------------------------------------------------------------------
    // Hardware
    // -----------------------------------------------------------------------

    driveTalon =
        new TalonFX(
            driveCanId,
            kCANBus);

    turnTalon =
        new TalonFX(
            turnCanId,
            kCANBus);

    turnEncoderAbsolute =
        new CANcoder(
            encoderCanId,
            kCANBus);


    // =========================================================================
    // DRIVE MOTOR
    // =========================================================================

    var driveConfig =
        new TalonFXConfiguration();

    driveConfig.MotorOutput.NeutralMode =
        NeutralModeValue.Brake;

    driveConfig.Feedback.SensorToMechanismRatio =
        driveMotorReduction;

    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        driveMotorCurrentLimit;

    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        -driveMotorCurrentLimit;

    driveConfig.CurrentLimits.StatorCurrentLimit =
        driveMotorCurrentLimit;

    driveConfig.CurrentLimits.StatorCurrentLimitEnable =
        true;

    driveConfig.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive;


    // -----------------------------------------------------------------------
    // Drive PID + feedforward
    // -----------------------------------------------------------------------

    var driveSlot0 =
        new Slot0Configs();

    driveSlot0.kP =
        driveKp;

    driveSlot0.kI =
        0.0;

    driveSlot0.kD =
        driveKd;

    driveSlot0.kS =
        driveKs;

    driveSlot0.kV =
        driveKv;

    driveSlot0.kA =
        driveKa;

    driveConfig.Slot0 =
        driveSlot0;


    // -----------------------------------------------------------------------
    // Apply drive configuration
    // -----------------------------------------------------------------------

    tryUntilOk(
        5,
        () ->
            driveTalon
                .getConfigurator()
                .apply(
                    driveConfig,
                    0.25));

    tryUntilOk(
        5,
        () ->
            driveTalon
                .setPosition(
                    0.0,
                    0.25));


    // =========================================================================
    // TURN MOTOR
    // =========================================================================

    var turnConfig =
        new TalonFXConfiguration();


    turnConfig.MotorOutput.NeutralMode =
        NeutralModeValue.Brake;


    // -----------------------------------------------------------------------
    // Steering feedback = CANcoder
    // -----------------------------------------------------------------------

    turnConfig.Feedback.FeedbackRemoteSensorID =
        encoderCanId;

    turnConfig.Feedback.FeedbackSensorSource =
        FeedbackSensorSourceValue.FusedCANcoder;

    turnConfig.Feedback.RotorToSensorRatio =
        turnMotorReduction;


    // -----------------------------------------------------------------------
    // Steering PID
    // -----------------------------------------------------------------------

    var turnSlot0 =
        new Slot0Configs();

    turnSlot0.kP =
        turnKp;

    turnSlot0.kI =
        0.0;

    turnSlot0.kD =
        turnKd;

    turnConfig.Slot0 =
        turnSlot0;


    // -----------------------------------------------------------------------
    // Steering motor direction
    // -----------------------------------------------------------------------

    turnConfig.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive;


    // -----------------------------------------------------------------------
    // Motion Magic
    // -----------------------------------------------------------------------

    turnConfig.MotionMagic.MotionMagicCruiseVelocity =
        100.0 / turnMotorReduction;

    turnConfig.MotionMagic.MotionMagicAcceleration =
        turnConfig
            .MotionMagic
            .MotionMagicCruiseVelocity
            / 0.100;


    // -----------------------------------------------------------------------
    // Allow steering to cross the 0/360 boundary
    // -----------------------------------------------------------------------

    turnConfig.ClosedLoopGeneral.ContinuousWrap =
        true;


    // -----------------------------------------------------------------------
    // Apply turn configuration
    // -----------------------------------------------------------------------

    tryUntilOk(
        5,
        () ->
            turnTalon
                .getConfigurator()
                .apply(
                    turnConfig,
                    0.25));


    // =========================================================================
    // CANCODER
    // =========================================================================

    var cancoderConfig =
        new CANcoderConfiguration();


    /*
     * Zero offset is handled using zeroRotation below.
     */
    cancoderConfig.MagnetSensor.MagnetOffset =
        0.0;

    cancoderConfig.MagnetSensor.SensorDirection =
        turnEncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;


    tryUntilOk(
        5,
        () ->
            turnEncoderAbsolute
                .getConfigurator()
                .apply(
                    cancoderConfig,
                    0.25));


    // =========================================================================
    // PHOENIX ODOMETRY
    // =========================================================================

    PhoenixOdometryThread odometryThread =
        PhoenixOdometryThread.getInstance();

    timestampQueue =
        odometryThread.makeTimestampQueue();


    // -----------------------------------------------------------------------
    // Drive signals
    // -----------------------------------------------------------------------

    drivePosition =
        driveTalon.getPosition();

    drivePositionQueue =
        odometryThread.registerSignal(
            drivePosition.clone());

    driveVelocity =
        driveTalon.getVelocity();

    driveAppliedVolts =
        driveTalon.getMotorVoltage();

    driveCurrent =
        driveTalon.getStatorCurrent();


    // -----------------------------------------------------------------------
    // Turn signals
    // -----------------------------------------------------------------------

    turnPosition =
        turnTalon.getPosition();

    turnPositionQueue =
        odometryThread.registerSignal(
            turnPosition.clone());

    turnVelocity =
        turnTalon.getVelocity();

    turnAppliedVolts =
        turnTalon.getMotorVoltage();

    turnCurrent =
        turnTalon.getStatorCurrent();


    // -----------------------------------------------------------------------
    // Absolute encoder
    // -----------------------------------------------------------------------

    turnAbsolutePosition =
        turnEncoderAbsolute.getAbsolutePosition();


    // =========================================================================
    // SIGNAL UPDATE FREQUENCIES
    // =========================================================================

    /*
     * These are the high-frequency signals used for odometry.
     */
    BaseStatusSignal.setUpdateFrequencyForAll(
        odometryFrequency,
        drivePosition,
        turnPosition);


    /*
     * These signals are only needed for normal robot inputs.
     */
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);


    // -----------------------------------------------------------------------
    // Optimize CAN utilization
    // -----------------------------------------------------------------------

    ParentDevice.optimizeBusUtilizationForAll(
        driveTalon,
        turnTalon,
        turnEncoderAbsolute);
  }


  // =========================================================================
  // INPUT UPDATE
  // =========================================================================

  @Override
  public void updateInputs(
      ModuleIOInputs inputs) {

    // -----------------------------------------------------------------------
    // Refresh drive signals
    // -----------------------------------------------------------------------

    var driveStatus =
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent);


    // -----------------------------------------------------------------------
    // Refresh steering signals
    // -----------------------------------------------------------------------

    var turnStatus =
        BaseStatusSignal.refreshAll(
            turnPosition,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent);


    // -----------------------------------------------------------------------
    // Refresh CANcoder
    // -----------------------------------------------------------------------

    var turnEncoderStatus =
        BaseStatusSignal.refreshAll(
            turnAbsolutePosition);


    // =========================================================================
    // DRIVE INPUTS
    // =========================================================================

    inputs.driveConnected =
        driveConnectedDebounce.calculate(
            driveStatus.isOK());


    inputs.drivePositionRad =
        Units.rotationsToRadians(
            drivePosition.getValueAsDouble());


    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(
            driveVelocity.getValueAsDouble());


    inputs.driveAppliedVolts =
        driveAppliedVolts.getValueAsDouble();


    inputs.driveCurrentAmps =
        driveCurrent.getValueAsDouble();


    // =========================================================================
    // TURN INPUTS
    // =========================================================================

    inputs.turnConnected =
        turnConnectedDebounce.calculate(
            turnStatus.isOK());


    /*
     * The TalonFX is configured to use the fused CANcoder
     * as its steering feedback source.
     *
     * This is the primary steering position used by the module.
     */
    inputs.turnPosition =
        Rotation2d.fromRotations(
                turnPosition.getValueAsDouble())
            .minus(
                zeroRotation);


    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(
            turnVelocity.getValueAsDouble());


    inputs.turnAppliedVolts =
        turnAppliedVolts.getValueAsDouble();


    inputs.turnCurrentAmps =
        turnCurrent.getValueAsDouble();


    // =========================================================================
    // ABSOLUTE CANCODER
    // =========================================================================

    /*
     * Keep the absolute CANcoder position separate from turnPosition.
     *
     * turnPosition is the fused TalonFX/CANcoder steering position.
     * turnAbsolutePosition is the raw absolute CANcoder measurement.
     */
    inputs.turnPosition =
        Rotation2d.fromRotations(
                turnAbsolutePosition.getValueAsDouble())
            .minus(
                zeroRotation);


    // =========================================================================
    // ODOMETRY
    // =========================================================================

    /*
     * Copy the high-frequency Phoenix samples into the module inputs.
     */

    inputs.odometryTimestamps =
        timestampQueue
            .stream()
            .mapToDouble(
                (Double value) -> value)
            .toArray();


    inputs.odometryDrivePositionsRad =
        drivePositionQueue
            .stream()
            .mapToDouble(
                (Double value) ->
                    Units.rotationsToRadians(value))
            .toArray();


    inputs.odometryTurnPositions =
        turnPositionQueue
            .stream()
            .map(
                (Double value) ->
                    Rotation2d
                        .fromRotations(value)
                        .minus(zeroRotation))
            .toArray(
                Rotation2d[]::new);


    // -----------------------------------------------------------------------
    // Clear queues after copying them
    // -----------------------------------------------------------------------

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }


  // =========================================================================
  // DRIVE OPEN LOOP
  // =========================================================================

  @Override
  public void setDriveOpenLoop(
      double output) {

    driveTalon.setControl(
        switch (driveClosedLoopOutput) {

          case Voltage ->
              voltageRequest
                  .withOutput(output);

          case TorqueCurrentFOC ->
              torqueCurrentRequest
                  .withOutput(output);
        });
  }


  // =========================================================================
  // TURN OPEN LOOP
  // =========================================================================

  @Override
  public void setTurnOpenLoop(
      double output) {

    turnTalon.setControl(
        switch (driveClosedLoopOutput) {

          case Voltage ->
              voltageRequest
                  .withOutput(output);

          case TorqueCurrentFOC ->
              torqueCurrentRequest
                  .withOutput(output);
        });
  }


  // =========================================================================
  // DRIVE VELOCITY
  // =========================================================================

  @Override
  public void setDriveVelocity(
      double velocityRadPerSec) {

    double velocityRotPerSec =
        Units.radiansToRotations(
            velocityRadPerSec);


    driveTalon.setControl(
        switch (driveClosedLoopOutput) {

          case Voltage ->
              velocityVoltageRequest
                  .withVelocity(
                      velocityRotPerSec);

          case TorqueCurrentFOC ->
              velocityTorqueCurrentRequest
                  .withVelocity(
                      velocityRotPerSec);
        });
  }


  // =========================================================================
  // TURN POSITION
  // =========================================================================

  @Override
  public void setTurnPosition(
      Rotation2d rotation) {

    /*
     * Add the module's physical zero offset before
     * sending the target to Phoenix.
     */
    Rotation2d target =
        rotation.plus(
            zeroRotation);


    turnTalon.setControl(
        switch (driveClosedLoopOutput) {

          case Voltage ->
              positionVoltageRequest
                  .withPosition(
                      target.getRotations());

          case TorqueCurrentFOC ->
              positionTorqueCurrentRequest
                  .withPosition(
                      target.getRotations());
        });
  }
}