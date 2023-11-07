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

  private double lastTurnVelocity = 0.0;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    turnFB.enableContinuousInput(-Math.PI, Math.PI);
    this.io.updateInputs(this.inputs);
    turnFB.reset(this.inputs.turnPositionAbsoluteRad);
  }

  public void periodic() {
    this.lastTurnVelocity = inputs.turnVelocityRadPerSec;
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);
  }

  public SwerveModuleState runSetpoint(SwerveModuleState state, boolean isStationary) {
    // Optimize state based on current angle
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

    io.setTurnVoltage(
        isStationary
            ? 0.0
            : turnFB.calculate(getAngle().getRadians(), optimizedState.angle.getRadians())
                + turnFF.calculate(turnFB.getSetpoint().velocity));

    // Update velocity based on turn error
    optimizedState.speedMetersPerSecond *= Math.cos(turnFB.getPositionError());

    io.setDriveVoltage(driveFF.calculate(optimizedState.speedMetersPerSecond));

    return optimizedState;
  }

  public void runDriveCharacterization(double volts) {
    io.setTurnVoltage(
        turnFB.calculate(getAngle().getRadians(), 0.0)
            + turnFB.calculate(turnFB.getSetpoint().velocity));
    io.setDriveVoltage(volts);
  }

  public void runTurnCharacterization(double volts) {
    io.setTurnVoltage(volts);
    io.setDriveVoltage(0);
  }

  public double getTurnVelocity() {
    return inputs.turnVelocityRadPerSec;
  }

  public double getTurnAcceleration() {
    return (inputs.turnVelocityRadPerSec - this.lastTurnVelocity) / 0.02;
  }

  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  public Rotation2d getAngle() {
    return new Rotation2d(MathUtil.angleModulus(inputs.turnPositionAbsoluteRad));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(inputs.drivePositionMeters, getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocityMetersPerSec, getAngle());
  }
}
