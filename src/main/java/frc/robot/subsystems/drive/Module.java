package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final SimpleMotorFeedforward driveFF =
      RobotConstants.get().moduleDriveFF().getFeedforward();
  private final SimpleMotorFeedforward turnFF =
      RobotConstants.get().moduleTurnFF().getFeedforward();
  private final ProfiledPIDController turnFB =
      RobotConstants.get()
          .moduleTurnFB()
          .getProfiledPIDController(new TrapezoidProfile.Constraints(550.6, 7585));

  private final double wheelRadius = (RobotConstants.get().moduleWheelDiameter() / 2);

  /**
   * Constructs a new Module object.
   *
   * @param io the ModuleIO instance for this module
   * @param index the index of this module
   */
  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    turnFB.enableContinuousInput(-Math.PI, Math.PI);
    this.io.updateInputs(this.inputs);
    turnFB.reset(this.inputs.turnPositionAbsoluteRad);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive/Module" + index, inputs);
  }

  /**
   * Runs the setpoint for the swerve module.
   *
   * @param state the desired swerve module state
   * @param isStationary indicates if the module is stationary
   * @return the optimized swerve module state after running the setpoint
   */
  public SwerveModuleState runSetpoint(SwerveModuleState state, boolean isStationary) {
    // Optimize state based on the current angle
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

    io.setTurnVoltage(
        isStationary
            ? 0.0
            : turnFB.calculate(getAngle().getRadians(), optimizedState.angle.getRadians())
                + turnFF.calculate(turnFB.getSetpoint().velocity));

    // Update velocity based on turn error
    optimizedState.speedMetersPerSecond *= Math.cos(turnFB.getPositionError());

    // Run drive controller
    double velocityRadPerSec = optimizedState.speedMetersPerSecond / wheelRadius;
    io.setDriveVoltage(driveFF.calculate(velocityRadPerSec));

    return optimizedState;
  }

  /**
   * Runs the characterization for the swerve module.
   *
   * @param volts the voltage to be applied for the characterization
   */
  public void runCharacterization(double volts) {
    io.setTurnVoltage(
        turnFB.calculate(getAngle().getRadians(), 0.0)
            + turnFB.calculate(turnFB.getSetpoint().velocity));
    io.setDriveVoltage(volts);
  }

  /** Stops the swerve module. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  /**
   * Sets the brake mode for the swerve module.
   *
   * @param enabled true to enable brake mode, false to disable brake mode
   */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /**
   * Returns the angle of the swerve module.
   *
   * @return the angle of the swerve module as a Rotation2d object
   */
  public Rotation2d getAngle() {
    return new Rotation2d(MathUtil.angleModulus(inputs.turnPositionAbsoluteRad));
  }

  /**
   * Returns the position of the swerve module in radians.
   *
   * @return the position of the swerve module in radians as a double value
   */
  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadius;
  }

  /**
   * Returns the velocity of the swerve module in radians per second.
   *
   * @return the velocity of the swerve module in radians per second as a double value
   */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadius;
  }

  /**
   * Returns the current position of the swerve module.
   *
   * @return the current position of the swerve module as a SwerveModulePosition object
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /**
   * Returns the current state of the swerve module.
   *
   * @return the current state of the swerve module as a SwerveModuleState object
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /**
   * Returns the characterization velocity of the swerve module.
   *
   * @return the characterization velocity of the swerve module as a double value
   */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
