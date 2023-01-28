package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private final CANSparkMax armExtendMotor;
  private final CANSparkMax armRotateMotor;

  private final Logger logger = Logger.getInstance();

  private static final SimpleMotorFeedforward extendFF =
      RobotConstants.get().moduleDriveFF().getFeedforward();
  private static final SimpleMotorFeedforward rotateFF =
      RobotConstants.get().moduleTurnFF().getFeedforward();

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged armIOInputs = new ArmIOInputsAutoLogged();

  private enum ArmMode {
    NORMAL,
    EXTEND_CHARACTERIZATION,
    ROTATE_CHARACTERIZATION
  }

  private ArmMode mode = ArmMode.NORMAL;

  private double angle = 0;
  private double characterizationVoltage = 0.0;
  private double distanceTargetInches = 0;
  private double rotateTargetDegrees = 0;
  private double extendSetpoint = 0.0;
  private double rotateSetpoint = 0.0;

  public static final double EXTEND_TOLERANCE_INCHES = 2.0;
  public static final double ROTATE_TOLERANCE_DEGREES = 2.0;

  private boolean enabled = false;

  private boolean isManual = true;
  private double manualExtend = 0.0;
  private double manualRotate = 0.0;

  /**
   * Configures the arm subsystem
   *
   * @param armIO Arm IO
   */
  public Arm(ArmIO armIO) {
    super();
    this.armIO = armIO;
    this.armExtendMotor = armIO.getExtendMotor();
    this.armRotateMotor = armIO.getRotateMotor();

    armIO.updateInputs(armIOInputs);
  }

  public void enable() {
    enabled = true;
  }

  public void disable() {
    stop();
    enabled = false;
  }

  public boolean isEnabled() {
    return enabled;
  }

  public boolean isManual() {
    return isManual;
  }

  public void stop() {
    isManual = true;
    armExtendMotor.set(0.0);
    armRotateMotor.set(0.0);
    manualExtend = 0;
    manualRotate = 0;
  }

  public void manualExtend(double extend) {
    isManual = true;
    manualExtend = extend;
  }

  public void manualRotate(double rotate) {
    isManual = true;
    manualRotate = rotate;
  }

  @Override
  public void periodic() {
    if (isManual) {
      armExtendMotor.set(manualExtend);
      armRotateMotor.set(manualRotate);
    }
    // Update inputs for IOs
    for (int i = 0; i < 4; i++) {
      armIO.updateInputs(armIOInputs);
      logger.processInputs("Arm", armIOInputs);
    }

    if (DriverStation.isDisabled()) {
      // Disable output while disabled
      armIO.setExtendVoltage(0.0);
      armIO.setRotateVoltage(0.0);
    } else if (isManual) {

    } else {
      switch (mode) {
        case NORMAL:
          // In normal mode, run the motors for arm extension and rotation
          // based on the current setpoint

          // Run extend controller
          double extendRadPerSec = 0.0;
          // TODO: Translate setpoint to voltage
          // setpointStatesOptimized[i].speedMetersPerSecond
          //     / (RobotConstants.get().moduleWheelDiameter() / 2);
          armIO.setExtendVoltage(extendFF.calculate(extendRadPerSec));

          // Run extend controller
          double rotateRadPerSec = 0.0;
          // TODO: Translate setpoint to voltage
          // setpointStatesOptimized[i].speedMetersPerSecond
          //     / (RobotConstants.get().moduleWheelDiameter() / 2);
          armIO.setRotateVoltage(rotateFF.calculate(rotateRadPerSec));

          // Log individual setpoints
          logger.recordOutput("ArmExtendSetpoint", extendRadPerSec);
          logger.recordOutput("ArmRotateSetpoint", rotateRadPerSec);
          break;

        case EXTEND_CHARACTERIZATION:
          armIO.setExtendVoltage(characterizationVoltage);
          break;

        case ROTATE_CHARACTERIZATION:
          armIO.setRotateVoltage(characterizationVoltage);
          break;
      }
    }

    // TODO: Translate velocity into movement
    double PLACEHOLDER = 1.0;
    // Log measured states
    logger.recordOutput("Arm/Extend/Velocity", armIOInputs.extendVelocity * PLACEHOLDER);
    logger.recordOutput("Arm/Extend/Position", armIOInputs.extendPosition * PLACEHOLDER);
    logger.recordOutput("Arm/Rotate/Velocity", armIOInputs.rotateVelocity * PLACEHOLDER);
    logger.recordOutput("Arm/Rotate/Position", armIOInputs.rotatePosition * PLACEHOLDER);

    // TODO: Proper conversions
    armExtendMotor.getEncoder().setPosition(distanceTargetInches);
    armRotateMotor.getEncoder().setPosition(rotateTargetDegrees);
  }

  public void setExtendSetpoint(double setpoint) {
    extendSetpoint = setpoint;
  }

  public void setRotateSetpoint(double setpoint) {
    rotateSetpoint = setpoint;
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

  public void extendAndRotate(double distanceTargetInches, double rotateTargetDegrees) {
    this.distanceTargetInches = distanceTargetInches;
    this.rotateTargetDegrees = rotateTargetDegrees;
  }

  public boolean isStopped() {
    if (armExtendMotor.getEncoder().getVelocity() <= 0.1
        && armRotateMotor.getEncoder().getVelocity() <= 0.1) return true;
    return false;
  }

  public boolean finished() {
    double currentDistance = armExtendMotor.getEncoder().getPosition(); // NEEDS CONVERSION
    // TODO: CHANGE to Lidar

    double currentAngle = armRotateMotor.getEncoder().getPosition(); // NEEDS CONVERSION

    if (currentDistance >= (distanceTargetInches - EXTEND_TOLERANCE_INCHES)
        && (currentDistance <= (distanceTargetInches + EXTEND_TOLERANCE_INCHES)
            && (currentAngle >= (rotateTargetDegrees - EXTEND_TOLERANCE_INCHES)
                && (currentAngle <= (rotateTargetDegrees + EXTEND_TOLERANCE_INCHES))))) {
      return true;
    }

    return false;
  }
}
