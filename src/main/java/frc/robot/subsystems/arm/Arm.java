package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private final Logger logger = Logger.getInstance();

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged armIOInputs = new ArmIOInputsAutoLogged();

  private enum ArmMode {
    AUTO,
    EXTEND_CHARACTERIZATION,
    ROTATE_CHARACTERIZATION,
    MANUAL,
    HOLD,
    CALIBRATE,
    KICKBACK,
  }

  private enum CalibrateMode {
    RETRACT_ARM,
    EXTEND_ARM,
    LOWER_ARM,
    RETRACT_ARM_AT_LOW_POSITION,
  }

  private enum AutoMode {
    RETRACT,
    RETRACT_FULL,
    ROTATE,
    EXTEND
  }

  private ArmMode mode = ArmMode.MANUAL;
  private AutoMode autoMode = AutoMode.RETRACT;
  private CalibrateMode calibrateMode = CalibrateMode.RETRACT_ARM;

  private Kickback kickback;

  private double characterizationVoltage = 0.0;
  private double extendSetpoint = 0.0;
  private double rotateSetpoint = 0.0;
  private boolean isCalibrated = false;
  private double calibrateRotateOrigin = 0;

  private Timer autoRetractTimer = new Timer();

  private static final double EXTEND_TOLERANCE_METERS = 0.008;
  private static final double ROTATE_TOLERANCE_METERS = 0.0023;
  // TODO: tweak all safety values, they are currently wrong
  private static final double SAFE_ROTATE_AT_FULL_EXTENSION = 0.0; // 0.069
  private static final double SAFE_EXTENSION_LENGTH = 0.0; // 0.02
  private static final double SAFE_ROTATE_AT_PARTIAL_EXTENSION = 0.0; // 0.026
  private static final double SAFE_EXTEND_AT_PARTIAL_EXTENSION = 0.0; // 0.229

  private static final double SAFE_RETRACT_NON_HOME = 0.05;

  private static final double EXTEND_CALIBRATION_POSITION = 0.01;
  private static final double ROTATE_RAISE_METERS = 0.025;

  private double holdPosition;
  private double manualExtendVolts = 0.0;
  private double manualRotateVolts = 0.0;
  private PIDController extendPidController = new PIDController(60, 0, 0);
  private PIDController rotatePidController = new PIDController(800, 0, 0);

  private static final double BACK_FORCE = -1.4; // -1.3
  private static final double HOLD_BACK_FORCE = -0.5;

  private static final double CALIBRATE_RETRACT_VOLTAGE = -1.5;
  private static final double CALIBRATE_ROTATE_VOLTAGE = -7;

  private static final double RETRACT_POSITION_CLOSE_TO_LIMIT = 0.1;
  private static final double RETRACT_VOLTAGE_CLOSE_TO_LIMIT = -0.7;

  /**
   * Constructs a new Arm object with the given ArmIO instance.
   *
   * @param armIO the ArmIO instance to be used by the Arm object
   */
  public Arm(ArmIO armIO) {
    super();
    this.armIO = armIO;

    armIO.updateInputs(armIOInputs);
  }

  /**
   * Returns whether the Arm object is currently in manual mode.
   *
   * @return true if the Arm object is in manual mode, false otherwise
   */
  public boolean isManual() {
    return mode == ArmMode.MANUAL;
  }

  /**
   * Stops the Arm object by setting it to manual mode and setting all voltage outputs to zero.
   *
   * <p>After calling this method, the Arm object will be in manual mode with zero extend and rotate
   * voltages.
   */
  public void stop() {
    mode = ArmMode.MANUAL;
    setExtendVoltage(0.0);
    setRotateVoltage(0.0);
    manualExtendVolts = 0;
    manualRotateVolts = 0;
  }

  /**
   * Sets the extend voltage of the Arm object to the specified value in manual mode.
   *
   * @param volts the voltage value to set for extend
   */
  public void manualExtend(double volts) {
    mode = ArmMode.MANUAL;
    manualExtendVolts = volts;
  }

  /**
   * Sets the rotate voltage of the Arm object to the specified value in manual mode.
   *
   * @param volts the voltage value to set for rotate
   */
  public void manualRotate(double volts) {
    mode = ArmMode.MANUAL;
    manualRotateVolts = volts;
  }

  /**
   * Initiates the calibration process for the Arm object.
   *
   * <p>This method sets the Arm mode to CALIBRATE and sets the calibration mode to RETRACT_ARM.
   *
   * <p>The calibration process is used to establish the starting position of the Arm's rotation.
   */
  public void calibrate() {
    calibrateMode = CalibrateMode.RETRACT_ARM;
    mode = ArmMode.CALIBRATE;
    calibrateRotateOrigin = armIOInputs.rotatePosition;
  }

  /** Zeros the positions of both motors, assuming that we're already at HOME position. */
  public void setCalibratedAssumeHomePosition() {
    armIO.resetExtendEncoderPosition();
    armIO.resetRotateEncoderPosition();
    isCalibrated = true;
    mode = ArmMode.AUTO;
  }

  /** Holds the current arm position by setting the target position to the current position. */
  public void hold() {
    hold(armIOInputs.extendPosition);
  }

  /**
   * Holds the current arm position by setting the target position to a given position.
   *
   * @param position The position to hold.
   */
  public void hold(double position) {
    mode = ArmMode.HOLD;
    holdPosition = position;
    setExtendVoltage(0.0);
    setRotateVoltage(0.0);
    manualRotateVolts = 0;
    manualExtendVolts = 0;
  }

  /**
   * Checks if the arm is currently holding its position.
   *
   * @return true if the arm is holding its position, false otherwise.
   */
  public boolean isHolding() {
    return mode == ArmMode.HOLD;
  }

  /** Raises the arm by a predetermined position. */
  public void raise() {
    if (armIOInputs.rotatePosition < 0.1) {
      mode = ArmMode.AUTO;
      autoMode = AutoMode.ROTATE;
      rotateSetpoint = armIOInputs.rotatePosition + ROTATE_RAISE_METERS;
      extendSetpoint = 0.02;
    }
  }

  /**
   * Determines if the arm should be raised.
   *
   * @return true if the arm should be raised, otherwise false.
   */
  public boolean shouldRaise() {
    return armIOInputs.rotatePosition < 0.1;
  }

  /**
   * Checks whether the arm is calibrated or not.
   *
   * @return true if the arm is calibrated, false otherwise.
   */
  public boolean isCalibrated() {
    return isCalibrated;
  }

  @Override
  public void periodic() {
    armIO.updateInputs(armIOInputs);
    logger.processInputs("Arm", armIOInputs);

    if (DriverStation.isDisabled()) {
      // Disable output while disabled
      stop();
      return;
    }
    logger.recordOutput("Arm/Mode", mode.toString());
    logger.recordOutput("Arm/CalibrateMode", calibrateMode.toString());
    logger.recordOutput("Arm/IsCalibrated", isCalibrated);

    switch (mode) {
      case MANUAL -> {
        if (isCalibrated
            && armIOInputs.extendPosition < RETRACT_POSITION_CLOSE_TO_LIMIT
            && manualExtendVolts < 0) {
          setExtendVoltage(Math.max(manualExtendVolts, RETRACT_VOLTAGE_CLOSE_TO_LIMIT));
        } else {
          setExtendVoltage(manualExtendVolts);
        }
        setRotateVoltage(manualRotateVolts);
        logger.recordOutput("Arm/ManualExtendVolts", manualRotateVolts);
        logger.recordOutput("Arm/ManualRotateVolts", manualRotateVolts);
      }
      case AUTO -> autoPeriodic();
      case EXTEND_CHARACTERIZATION -> setExtendVoltage(characterizationVoltage);
      case ROTATE_CHARACTERIZATION -> setRotateVoltage(characterizationVoltage);
      case HOLD -> armIO.setExtendVoltageWhileHold(
          calculateExtendPid(holdPosition) + HOLD_BACK_FORCE);
      case CALIBRATE -> calibratePeriodic();
      case KICKBACK -> kickback.periodic();
    }
  }

  private void autoPeriodic() {
    if (isFinished()) {
      // Reached target.
      hold();
      return;
    }
    logger.recordOutput("Arm/AutoMode", autoMode.toString());
    switch (autoMode) {
      case RETRACT -> {
        if (autoRetractTimer.hasElapsed(1)
            || armIOInputs.extendPosition < SAFE_RETRACT_NON_HOME + EXTEND_TOLERANCE_METERS) {
          setExtendVoltage(0);
          autoMode = rotateSetpoint > 0.013 ? AutoMode.ROTATE : AutoMode.RETRACT_FULL;
        } else {
          setExtendVoltage(calculateExtendPid(SAFE_RETRACT_NON_HOME));
        }
      }
      case RETRACT_FULL -> {
        if (autoRetractTimer.hasElapsed(2.0) // 1.5
            || armIOInputs.extendPosition
                < RobotConstants.get().armExtendMinMeters() + EXTEND_TOLERANCE_METERS) {
          setExtendVoltage(0);
          autoMode = AutoMode.ROTATE;
        } else {
          setExtendVoltage(calculateExtendPid(RobotConstants.get().armExtendMinMeters()));
        }
      }
      case ROTATE -> {
        setRotateVoltage(calculateRotatePid(rotateSetpoint));
        if (isRotateFinished()) {
          autoMode = AutoMode.EXTEND;
          setRotateVoltage(0);
        }
      }
      case EXTEND -> {
        double extendFbOutput = calculateExtendPid(extendSetpoint);
        setExtendVoltage(extendFbOutput);
      }
    }

    logger.recordOutput("Arm/ExtendSetpoint", extendSetpoint);
    logger.recordOutput("Arm/RotateSetpoint", rotateSetpoint);
  }

  private void calibratePeriodic() {
    switch (calibrateMode) {
      case RETRACT_ARM -> {
        // Drive Extend Motor until hit limit switch
        if (armIOInputs.extendReverseLimitSwitch && armIOInputs.rotateLowLimitSwitch) {
          armIO.resetExtendEncoderPosition();
          armIO.resetRotateEncoderPosition();
          isCalibrated = true;
          hold();
          break;
        }
        if (armIOInputs.extendReverseLimitSwitch) {
          armIO.resetExtendEncoderPosition();
          calibrateMode = CalibrateMode.EXTEND_ARM;
        } else {
          setExtendVoltage(CALIBRATE_RETRACT_VOLTAGE);
        }
      }
      case EXTEND_ARM -> {
        // Drive Extend Motor a little bit outwards
        setExtendVoltage(calculateExtendPidUnsafe(EXTEND_CALIBRATION_POSITION));
        if (isExtendPositionNear(EXTEND_CALIBRATION_POSITION)) {
          calibrateMode = CalibrateMode.LOWER_ARM;
          setExtendVoltage(0);
        }
      }
      case LOWER_ARM -> {
        // Drive rotate motor until hit lower limit switch
        if (calibrateRotateOrigin - armIOInputs.rotatePosition > 0.15) {
          // If we've travelled a lot, start over so that we don't pull the belt too far.
          calibrate();
        } else if (armIOInputs.rotateLowLimitSwitch) {
          armIO.resetRotateEncoderPosition();
          armIO.setRotateVoltage(0);
          calibrateMode = CalibrateMode.RETRACT_ARM_AT_LOW_POSITION;
        } else {
          setRotateVoltage(CALIBRATE_ROTATE_VOLTAGE);
        }
      }
      case RETRACT_ARM_AT_LOW_POSITION -> {
        // Drive Extend Motor until hit limit switch
        if (armIOInputs.extendReverseLimitSwitch) {
          armIO.resetExtendEncoderPosition();
          isCalibrated = true;
          hold();
        } else {
          setExtendVoltage(CALIBRATE_RETRACT_VOLTAGE);
        }
      }
    }
  }

  /**
   * Sets the target positions for extending and rotating the arm.
   *
   * @param extendSetpoint The target position for extending the arm.
   * @param rotateSetpoint The target position for rotating the arm.
   */
  public void setTargetPositions(double extendSetpoint, double rotateSetpoint) {
    mode = ArmMode.AUTO;
    autoMode = AutoMode.RETRACT;
    autoRetractTimer.reset();
    autoRetractTimer.start();
    this.extendSetpoint =
        MathUtil.clamp(
            extendSetpoint,
            RobotConstants.get().armExtendMinMeters(),
            RobotConstants.get().armExtendMaxMeters());
    this.rotateSetpoint =
        MathUtil.clamp(
            rotateSetpoint,
            RobotConstants.get().armRotateMinMeters(),
            RobotConstants.get().armRotateMaxMeters());
  }

  /**
   * Sets the target position for extending the arm.
   *
   * @param extendSetpoint The target position for extending the arm.
   */
  public void setTargetPositionExtend(double extendSetpoint) {
    setTargetPositions(extendSetpoint, armIOInputs.rotatePosition);
  }

  /**
   * Sets the target position for rotating the arm.
   *
   * @param rotateSetpoint The target position for rotating the arm.
   */
  public void setTargetPositionRotate(double rotateSetpoint) {
    setTargetPositions(armIOInputs.extendPosition, rotateSetpoint);
    autoMode = AutoMode.ROTATE;
  }

  public void characterizeExtend() {
    mode = ArmMode.EXTEND_CHARACTERIZATION;
  }

  public void characterizeRotate() {
    mode = ArmMode.ROTATE_CHARACTERIZATION;
  }

  public void runCharacterizationVolts(double volts) {
    characterizationVoltage = volts;
  }

  public double getCharacterizationVelocity() {
    if (mode == ArmMode.EXTEND_CHARACTERIZATION) {
      return armIOInputs.extendVelocity;
    } else if (mode == ArmMode.ROTATE_CHARACTERIZATION) {
      return armIOInputs.rotateVelocity;
    } else {
      return 0.0;
    }
  }

  /**
   * Retrieves the current rotation position of the arm.
   *
   * @return The current rotation position of the arm.
   */
  public double getRotation() {
    return armIOInputs.rotatePosition;
  }

  /**
   * Retrieves the current extension position of the arm.
   *
   * @return The current extension position of the arm.
   */
  public double getExtention() {
    return armIOInputs.extendPosition;
  }

  /**
   * Checks if the arm is stopped.
   *
   * <p>The arm is considered stopped if it is in MANUAL mode and both the manualExtendVolts and
   * manualRotateVolts are equal to 0.
   *
   * @return true if the arm is stopped, false otherwise.
   */
  public boolean isStopped() {
    return mode == ArmMode.MANUAL && manualExtendVolts == 0 && manualRotateVolts == 0;
  }

  public boolean isFinished() {
    return isExtendPositionNear(extendSetpoint) && isRotateFinished();
  }

  /**
   * Checks if the rotation of the arm is finished.
   *
   * <p>The rotation is considered finished if any of the following conditions are met:
   *
   * <ul>
   *   <li>The absolute difference between the current rotate position and the rotate setpoint is
   *       less than or equal to the rotation tolerance in meters.
   *   <li>The rotate high limit switch is activated and the current rotate position is less than
   *       the rotate setpoint.
   *   <li>The rotate low limit switch is activated and the current rotate position is greater than
   *       the rotate setpoint.
   * </ul>
   *
   * @return true if the rotation of the arm is finished, false otherwise.
   */
  public boolean isRotateFinished() {
    return Math.abs(armIOInputs.rotatePosition - rotateSetpoint) <= ROTATE_TOLERANCE_METERS
        || (armIOInputs.rotateHighLimitSwitch && armIOInputs.rotatePosition < rotateSetpoint)
        || (armIOInputs.rotateLowLimitSwitch && armIOInputs.rotatePosition > rotateSetpoint);
  }

  /**
   * Sets the voltage for rotating the arm.
   *
   * @param volts the voltage value to set for rotating the arm.
   */
  private void setRotateVoltage(double volts) {
    armIO.setRotateVoltage(volts);
  }

  /**
   * Sets the voltage for extending the arm.
   *
   * @param volts the voltage value to set for extending the arm. Positive voltages extend the arm,
   *     negative voltages retract the arm. A voltage of 0 stops the arm from extending or
   *     retracting.
   */
  private void setExtendVoltage(double volts) {
    if (!maybeKickback(volts)) {
      if (volts == 0) {
        armIO.setExtendVoltage(0);
      } else if (volts < 0) {
        armIO.setExtendVoltage(volts + BACK_FORCE);
      } else {
        armIO.setExtendVoltage(volts);
      }
    }
  }

  /**
   * Calculates the PID value for extending the arm based on the target position.
   *
   * @param targetPosition the target position to extend the arm.
   * @return the PID value calculated for extending the arm. Returns 0 if the target position is not
   *     safe to extend.
   */
  private double calculateExtendPid(double targetPosition) {
    if (!isExtendSafe(targetPosition)) {
      return 0;
    }
    return calculateExtendPidUnsafe(targetPosition);
  }

  /**
   * Calculates the PID value for extending the arm.
   *
   * @param targetPosition the desired position for extending the arm.
   * @return the PID value calculated for extending the arm.
   */
  private double calculateExtendPidUnsafe(double targetPosition) {
    double pidValue = extendPidController.calculate(armIOInputs.extendPosition, targetPosition);
    logger.recordOutput("Arm/ExtendFbOutput", pidValue);
    return pidValue;
  }

  /**
   * Calculates the PID value for rotating the arm.
   *
   * @param targetPosition the desired position for rotating the arm.
   * @return the PID value calculated for rotating the arm. Returns 0 if it is not safe to rotate to
   *     the target position.
   */
  private double calculateRotatePid(double targetPosition) {
    if (!isRotateSafe(targetPosition)) {
      return 0;
    }
    double pidValue = rotatePidController.calculate(armIOInputs.rotatePosition, targetPosition);
    logger.recordOutput("Arm/RotateFbOutput", pidValue);
    return pidValue;
  }

  /**
   * Determines if the target position is near the extend position.
   *
   * @param targetPosition the desired position to check for proximity.
   * @return true if the target position is near the extend position within the tolerance defined by
   *     EXTEND_TOLERANCE_METERS, false otherwise.
   */
  private boolean isExtendPositionNear(double targetPosition) {
    return Math.abs(armIOInputs.extendPosition - targetPosition) <= EXTEND_TOLERANCE_METERS;
  }

  /**
   * Checks if it is safe to rotate the arm to the specified position.
   *
   * @param rotatePosition the position to which the arm intends to rotate.
   * @return true if it is safe to rotate to the target position, false otherwise.
   */
  private boolean isRotateSafe(double rotatePosition) {
    boolean isSafe =
        isCalibrated
            && (rotatePosition > armIOInputs.rotatePosition
                || rotatePosition > SAFE_ROTATE_AT_FULL_EXTENSION
                || armIOInputs.extendPosition < SAFE_EXTENSION_LENGTH
                || (rotatePosition > SAFE_ROTATE_AT_PARTIAL_EXTENSION - ROTATE_TOLERANCE_METERS
                    && armIOInputs.extendPosition
                        < SAFE_EXTEND_AT_PARTIAL_EXTENSION - EXTEND_TOLERANCE_METERS));
    logger.recordOutput("Arm/IsRotateSafe", isSafe);
    return isSafe;
  }

  /**
   * Checks if it is safe to extend the arm to the specified position.
   *
   * @param targetPosition the position to which the arm intends to extend.
   * @return true if it is safe to extend to the target position, false otherwise.
   */
  private boolean isExtendSafe(double targetPosition) {
    boolean isSafe =
        isCalibrated
            && (targetPosition < armIOInputs.extendPosition
                || armIOInputs.rotatePosition > SAFE_ROTATE_AT_FULL_EXTENSION
                || (armIOInputs.rotatePosition > SAFE_ROTATE_AT_PARTIAL_EXTENSION
                    && targetPosition < SAFE_EXTEND_AT_PARTIAL_EXTENSION));
    logger.recordOutput("Arm/IsExtendSafe", isSafe);
    return isSafe;
  }

  /**
   * Checks if it is possible to initiate a kickback movement of the arm to the specified target
   * volts.
   *
   * @param targetVolts the voltage to which the arm intends to kickback.
   * @return true if the kickback can be initiated, false otherwise.
   */
  private boolean maybeKickback(double targetVolts) {
    if (mode != ArmMode.KICKBACK && targetVolts > 0 && armIOInputs.ratchetLocked) {
      kickback = new Kickback(targetVolts);
      mode = ArmMode.KICKBACK;
      return true;
    }
    return false;
  }

  private class Kickback {
    private final ArmMode returnToMode;
    private final double targetVolts;
    private final Timer timer = new Timer();

    Kickback(double targetVolts) {
      returnToMode = mode;
      armIO.setExtendVoltage(-2);
      this.targetVolts = targetVolts;
      timer.reset();
      timer.start();
    }

    void periodic() {
      if (timer.hasElapsed(0.1)) {
        setExtendVoltage(targetVolts);
        mode = returnToMode;
        kickback = null;
      } else {
        armIO.setExtendVoltage(-2);
      }
    }
  }
}
