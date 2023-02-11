package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
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
    CALIBRATE
  }

  private ArmMode mode = ArmMode.CALIBRATE;
  private double holdPosition;
  private double angle = 0;
  private double characterizationVoltage = 0.0;
  private double extendSetpoint = 0.2;
  private double rotateSetpoint = 0.0;

  private boolean extendCalibrated = false;
  private boolean rotationCalibrated = true;

  private static final double EXTEND_TOLERANCE_METERS = 0.005;
  private static final double ROTATE_TOLERANCE_DEGREES = 2.0;

  private double manualExtend = 0.0;
  private double manualRotate = 0.0;
  private PIDController pidController = new PIDController(20, 0, 0);
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
    holdPosition = armIOInputs.extendPosition;
    mode = ArmMode.HOLD;
  }

  public void hold(double position) {
    holdPosition = position;
    mode = ArmMode.HOLD;
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

    if (armIO.isExtendLimitSwitchPressed() && mode != ArmMode.CALIBRATE) {
      if (Math.abs(armIOInputs.extendPosition) > 0.1) {
        mode = ArmMode.CALIBRATE;
        extendCalibrated = false;
      } else {
        hold();
      }
    }
    armIO.updateInputs(armIOInputs);
    logger.processInputs("Arm", armIOInputs);
    logger.recordOutput("Arm/mode", mode.toString());

    switch (mode) {
      case MANUAL:
        if (armIOInputs.extendPosition > RobotConstants.get().armExtendMax() && manualExtend > 0) {
          armIO.setExtendVelocity(0);
        } else if (armIOInputs.extendPosition < RobotConstants.get().armExtendMin()
            && manualExtend < 0) {
          hold();
        } else {
          armIO.setExtendVelocity(manualExtend);
        }
        armIO.setRotateVelocity(manualRotate);
        break;

      case AUTO:
        if (Math.abs(armIOInputs.extendPosition - extendSetpoint) <= EXTEND_TOLERANCE_METERS) {
          // Reached target.
          hold();
        } else {
          double fbOutput = calculateExtendPid(extendSetpoint);
          logger.recordOutput("Arm/fbOutput", fbOutput);
          armIO.setExtendVoltage(fbOutput);
        }
        logger.recordOutput("ArmExtendSetpoint", extendSetpoint);
        logger.recordOutput("ArmRotateSetpoint", rotateSetpoint);
        break;

      case EXTEND_CHARACTERIZATION:
        armIO.setExtendVoltage(characterizationVoltage);
        break;

      case ROTATE_CHARACTERIZATION:
        armIO.setRotateVoltage(characterizationVoltage);
        break;

      case DISABLED:
        break;

      case HOLD:
        double holdPidOutput = calculateExtendPid(holdPosition);
        logger.recordOutput("Arm/holdPidOutput", holdPidOutput);
        if (armIO.isExtendLimitSwitchPressed()) {
          armIO.setExtendVoltage(0);
        } else {
          armIO.setExtendVoltage(holdPidOutput);
        }
        break;

      case CALIBRATE:
        if (armIO.isExtendLimitSwitchPressed()) {
          if (!extendCalibrated) {
            armIO.resetEncoderPosition();
          }
          armIO.setExtendVoltage(calculateExtendPid(0));
          extendCalibrated = true;
        } else {
          armIO.setExtendVoltage(-1);
        }
        if (extendCalibrated && rotationCalibrated) {
          hold(0);
        }
        break;
    }
    armIO.setRatchetLocked(isHolding());
  }

  public void setTargetPositions(double extendSetpoint, double rotateSetpoint) {
    mode = ArmMode.AUTO;
    this.extendSetpoint = Math.min(RobotConstants.get().armExtendMax(), extendSetpoint);
    this.rotateSetpoint = rotateSetpoint;
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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  public boolean isStopped() {
    if (armIOInputs.extendVelocity <= 0.1 && armIOInputs.rotateVelocity <= 0.1) return true;
    return false;
  }

  public boolean finished() {
    return Math.abs(armIOInputs.extendPosition - extendSetpoint) <= EXTEND_TOLERANCE_METERS;
  }

  public void resetEncoderPosition() {
    armIO.resetEncoderPosition();
  }

  private double calculateExtendPid(double targetPosition) {
    return pidController.calculate(armIOInputs.extendPosition, targetPosition) + BACK_FORCE;
  }
}
