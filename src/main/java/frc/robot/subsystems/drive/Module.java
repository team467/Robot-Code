package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private SimpleMotorFeedforward driveFF;
  private final PIDController turnFB;

  private final double wheelRadius = (RobotConstants.get().moduleWheelDiameter() / 2);

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    this.driveFF =
        new SimpleMotorFeedforward(
            RobotConstants.get().moduleDriveFF().getkS(),
            RobotConstants.get().moduleDriveFF().getkV(),
            RobotConstants.get().moduleDriveFF().getkA());
    this.turnFB =
        new PIDController(
            RobotConstants.get().moduleTurnFB().getkP(),
            0,
            RobotConstants.get().moduleTurnFB().getkD());

    turnFB.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive/Module" + index, inputs);

    if (RobotConstants.get().moduleDriveFF().hasChanged(hashCode())) {
      this.driveFF =
          new SimpleMotorFeedforward(
              RobotConstants.get().moduleDriveFF().getkS(),
              RobotConstants.get().moduleDriveFF().getkV(),
              RobotConstants.get().moduleDriveFF().getkA());
    }
    if (RobotConstants.get().moduleTurnFB().hasChanged(hashCode())) {
      turnFB.setPID(
          RobotConstants.get().moduleTurnFB().getkP(),
          0.0,
          RobotConstants.get().moduleTurnFB().getkD());
    }
  }

  public SwerveModuleState runSetpoint(SwerveModuleState state, boolean isStationary) {
    // Optimize state based on current angle
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

    io.setTurnVoltage(
        isStationary
            ? 0.0
            : turnFB.calculate(getAngle().getRadians(), optimizedState.angle.getRadians()));

    // Update velocity based on turn error
    optimizedState.speedMetersPerSecond *= Math.cos(turnFB.getPositionError());

    // Run drive controller
    double velocityRadPerSec = optimizedState.speedMetersPerSecond / wheelRadius;
    io.setDriveVoltage(driveFF.calculate(velocityRadPerSec));

    return optimizedState;
  }

  public void runCharacterization(double volts) {
    io.setTurnVoltage(turnFB.calculate(getAngle().getRadians(), 0.0));
    io.setDriveVoltage(volts);
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

  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadius;
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadius;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
