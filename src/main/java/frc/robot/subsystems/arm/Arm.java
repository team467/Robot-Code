package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
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
    DISABLED,
    HOLD,
    CALIBRATE,
  }

  private enum CalibrateMode {
    PHASE_ONE,
    PHASE_TWO,
    PHASE_THREE,
    PHASE_FOUR,
  }

  private ArmMode mode = ArmMode.CALIBRATE;
  private CalibrateMode calibrateMode = CalibrateMode.PHASE_ONE;
  private double holdPosition;
  private double characterizationVoltage = 0.0;
  private double extendSetpoint = 0.2;
  private double rotateSetpoint = 0.0;
  private boolean isCalibrated = false;

  private boolean hasRotate = true;
  private boolean hasExtend = false;
  private static final double EXTEND_TOLERANCE_METERS = 0.005;
  private static final double ROTATE_TOLERANCE_DEGREES = 2.0;

  private static final double EXTEND_CALIBRATION_POSITION = 0.05;

  private double manualExtend = 0.0;
  private double manualRotate = 0.0;
  private PIDController extendPidController = new PIDController(20, 0, 0);
  private PIDController rotatePidController = new PIDController(7, 0, 0);
  private static final double BACK_FORCE = -0.25;

  /**
   * Configures the arm subsystem
   *
   * @param armIO Arm IO
   */
  public Arm(ArmIO armIO) {
    super();
    this.armIO = armIO;

    armIO.updateInputs(armIOInputs);
  }

  public boolean isManual() {
    return mode == ArmMode.MANUAL;
  }

  public void stop() {
    mode = ArmMode.MANUAL;
    armIO.setExtendVoltage(0.0);
    armIO.setRotateVoltage(0.0);
    manualExtend = 0;
    manualRotate = 0;
  }

  public void manualExtend(double direction) {
    mode = ArmMode.MANUAL;
    manualExtend = direction;
  }

  public void manualRotate(double direction) {
    mode = ArmMode.MANUAL;
    manualRotate = direction;
  }

  public void hold() {
    hold(armIOInputs.extendPosition);
  }

  public void hold(double position) {
    holdPosition = position;
    mode = ArmMode.HOLD;
    armIO.setExtendVoltage(0.0);
    armIO.setRotateVoltage(0.0);
    manualRotate = 0;
    manualExtend = 0;
  }

  public boolean isHolding() {
    return mode == ArmMode.HOLD;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      // Disable output while disabled
      armIO.setExtendVoltage(0.0);
      armIO.setRotateVoltage(0.0);
      return;
    }

    armIO.updateInputs(armIOInputs);
    logger.processInputs("Arm", armIOInputs);
    logger.recordOutput("Arm/Mode", mode.toString());
    logger.recordOutput("Arm/CalibrateMode", calibrateMode.toString());

    switch (mode) {
      case MANUAL:
        if (armIOInputs.extendPosition > RobotConstants.get().armExtendMaxMeters()
            && manualExtend > 0) {
          armIO.setExtendVelocity(0);
        } else if (armIOInputs.extendPosition < RobotConstants.get().armExtendMinMeters()
            && manualExtend < 0) {
          armIO.setExtendVoltage(calculateExtendPid(RobotConstants.get().armExtendMinMeters()));
        } else {
          armIO.setExtendVelocity(manualExtend);
        }
        if (hasRotate) {
          if (armIOInputs.rotatePosition > RobotConstants.get().armRotateMaxMeters()
              && manualRotate > 0) {
            armIO.setRotateVelocity(0);
          } else if (armIOInputs.rotatePosition < RobotConstants.get().armRotateMinMeters()
              && manualRotate < 0) {
            armIO.setRotateVoltage(calculateRotatePid(RobotConstants.get().armRotateMinMeters()));
          } else {
            rotate(manualRotate);
          }
        }
        logger.recordOutput("Arm/ManualRotate", manualRotate);
        break;

      case AUTO:
        if (finished()) {
          // Reached target.
          hold();
        } else {
          double extendFbOutput = calculateExtendPid(extendSetpoint);
          double rotateFbOutput = calculateRotatePid(rotateSetpoint);

          logger.recordOutput("Arm/ExtendFbOutput", extendFbOutput);
          logger.recordOutput("Arm/RotateFbOutput", rotateFbOutput);
          armIO.setExtendVoltage(extendFbOutput);
          rotate(rotateFbOutput);
        }
        logger.recordOutput("Arm/ExtendSetpoint", extendSetpoint);
        logger.recordOutput("Arm/RotateSetpoint", rotateSetpoint);
        break;
      case EXTEND_CHARACTERIZATION:
        armIO.setExtendVoltage(characterizationVoltage);
        break;
      case ROTATE_CHARACTERIZATION:
        rotate(characterizationVoltage);

        break;
      case DISABLED:
        break;
      case HOLD:
        armIO.setExtendVoltage(0);
        break;
      case CALIBRATE:
        calibrate();

        break;
    }
    armIO.setRatchetLocked(isHolding());
  }

  private void calibrate() {
    switch (calibrateMode) {
      case PHASE_ONE:
        // Drive Extend Motor until hit limit switch
        if (armIOInputs.extendLimitSwitch && armIOInputs.rotateLowLimitSwitch) {
          hold();
        }
        if (hasExtend) {
          if (armIOInputs.extendLimitSwitch) {
            armIO.resetEncoderPosition();
            calibrateMode = CalibrateMode.PHASE_TWO;
          } else {
            armIO.setExtendVoltage(-1);
          }
        } else {
          calibrateMode = CalibrateMode.PHASE_THREE;
        }

        break;
      case PHASE_TWO:
        // Drive Extend Motor a little bit outwards
        if (hasExtend) {
          armIO.setExtendVoltage(calculateExtendPid(EXTEND_CALIBRATION_POSITION));
          if (Math.abs(armIOInputs.extendPosition - EXTEND_CALIBRATION_POSITION)
              <= EXTEND_TOLERANCE_METERS) {
            calibrateMode = CalibrateMode.PHASE_THREE;
          }
        }
        break;
      case PHASE_THREE:
        // Drive rotate motor until hit lower limit switch
        if (hasRotate) {
          armIO.setExtendVoltage(calculateExtendPid(EXTEND_CALIBRATION_POSITION));
          if (armIOInputs.rotateLowLimitSwitch) {
            armIO.resetRotateEncoder();
            armIO.setRotateVoltage(0);
            calibrateMode = CalibrateMode.PHASE_FOUR;
          } else {
            armIO.setRotateVoltage(-3);
          }
        } else {
          calibrateMode = CalibrateMode.PHASE_FOUR;
        }

        break;
      case PHASE_FOUR:
        // Drive Extend Motor until hit limit switch
        if (hasExtend) {
          if (armIOInputs.extendLimitSwitch) {
            armIO.resetEncoderPosition();
            hold();
            isCalibrated = true;
          } else {
            armIO.setExtendVoltage(-1);
          }
        } else {
          hold();
          isCalibrated = true;
        }
        break;
    }
  }

  public void setTargetPositions(double extendSetpoint, double rotateSetpoint) {
    mode = ArmMode.AUTO;
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

  public boolean isStopped() {
    return mode == ArmMode.MANUAL && manualExtend == 0 && manualRotate == 0;
  }

  public boolean finished() {
    return ((!hasExtend
            || Math.abs(armIOInputs.extendPosition - extendSetpoint) <= EXTEND_TOLERANCE_METERS)
        && ((!hasRotate
            || Math.abs(armIOInputs.rotatePosition - rotateSetpoint) <= ROTATE_TOLERANCE_DEGREES)));
  }

  private void rotate(double volts) {
    armIO.setRotateVoltage(volts);
  }

  public void resetEncoderPosition() {
    armIO.resetEncoderPosition();
  }

  private double calculateExtendPid(double targetPosition) {
    if (!isExtendSafe(targetPosition)) {
      return 0;
    }
    return (extendPidController.calculate(armIOInputs.extendPosition, targetPosition) + BACK_FORCE);
  }

  private double calculateRotatePid(double targetPosition) {
    if (!isRotateSafe(targetPosition)) {
      return 0;
    }
    return rotatePidController.calculate(armIOInputs.rotatePosition, targetPosition);
  }

  private boolean isRotateSafe(double rotatePosition) {
    return isCalibrated && (rotatePosition > 0.2 || armIOInputs.extendPosition < 0.1);
  }

  private boolean isExtendSafe(double extendPosition) {
    return isCalibrated && armIOInputs.rotatePosition > 0.2;
  }
}
