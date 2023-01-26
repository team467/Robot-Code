package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

	private final Logger logger = Logger.getInstance();

	private static final SimpleMotorFeedforward extendFF = RobotConstants.get().moduleDriveFF().getFeedforward(); 
	private static final SimpleMotorFeedforward rotateFF = RobotConstants.get().moduleTurnFF().getFeedforward();

	private final ModuleIO armIO;
	private final ModuleIOInputsAutoLogged armIOInputs = new ModuleIOInputsAutoLogged();

	private enum ArmMode {
		NORMAL,
		EXTEND_CHARACTERIZATION,
		ROTATE_CHARACTERIZATION
	}
	private ArmMode mode = ArmMode.NORMAL;

	private double angle = 0;
	private double characterizationVoltage = 0.0;

	private double extendSetpoint = 0.0;
	private double rotateSetpoint = 0.0;

	/**
	 * Configures the arm subsystem
	 *
	 * @param armIO   Arm IO
	 */
	public Arm(ModuleIO armIO) {
		super();
		this.armIO = armIO;
		armIO.updateInputs(armIOInputs);
	}

	@Override
  public void periodic() {

    // Update inputs for IOs
    for (int i = 0; i < 4; i++) {
      armIO.updateInputs(armIOInputs);
      logger.processInputs("Arm", armIOInputs);
    }

    if (DriverStation.isDisabled()) {
      // Disable output while disabled
      for (int i = 0; i < 4; i++) {
        armIO.setExtendVoltage(0.0);
        armIO.setRotateVoltage(0.0);
      }
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

}
