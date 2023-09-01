package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.autocheck.FaultReporter;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class Module {
  private String subsystemName;
  private CommandBase wrappedSystemCheckCommand;

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

  public Module(ModuleIO io, int index) {
    this.subsystemName = "Module" + index;

    this.io = io;
    this.index = index;

    turnFB.enableContinuousInput(-Math.PI, Math.PI);
    this.io.updateInputs(this.inputs);
    turnFB.reset(this.inputs.turnPositionAbsoluteRad);

    this.wrappedSystemCheckCommand =
        FaultReporter.getInstance()
            .registerSystemCheck(this.subsystemName, getSystemCheckCommand());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive/Module" + index, inputs);
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

    // Run drive controller
    double velocityRadPerSec = optimizedState.speedMetersPerSecond / wheelRadius;
    io.setDriveVoltage(driveFF.calculate(velocityRadPerSec));

    return optimizedState;
  }

  public void runCharacterization(double volts) {
    io.setTurnVoltage(
        turnFB.calculate(getAngle().getRadians(), 0.0)
            + turnFB.calculate(turnFB.getSetpoint().velocity));
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

  private CommandBase getSystemCheckCommand() {
    return Commands.sequence(
            Commands.run(
                    () -> runSetpoint(new SwerveModuleState(0, Rotation2d.fromDegrees(90)), false))
                .withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (Units.radiansToDegrees(inputs.turnPositionRad) < 70
                      || Units.radiansToDegrees(inputs.turnPositionRad) > 110) {
                    FaultReporter.getInstance()
                        .addFault(
                            this.subsystemName,
                            "[System Check] Rotation Motor did not reach target position",
                            false,
                            true);
                  }
                }),
            Commands.waitSeconds(0.25),
            Commands.run(
                    () -> runSetpoint(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false))
                .withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (Units.radiansToDegrees(inputs.turnPositionRad) < -20
                      || Units.radiansToDegrees(inputs.turnPositionRad) > 20) {
                    FaultReporter.getInstance()
                        .addFault(
                            this.subsystemName,
                            "[System Check] Rotation Motor did not reach target position",
                            false,
                            true);
                  }
                }))
        .until(() -> !FaultReporter.getInstance().getFaults(this.subsystemName).isEmpty())
        .andThen(Commands.runOnce(() -> io.setDriveVoltage(0.0)));
  }

  public CommandBase getCheckCommand() {
    return this.wrappedSystemCheckCommand;
  }
}
