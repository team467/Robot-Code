package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  /** Creates a new shooter. */
  private final ShooterIO io;

  private final RobotState robotState = RobotState.getInstance();
  private boolean PIDMode = false;
  private double currentVelocitySetpoint;
  private SimpleMotorFeedforward shooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.SHOOTER_KS.get(), ShooterConstants.SHOOTER_KV.get());
  private PIDController shooterFeedback =
      new PIDController(ShooterConstants.SHOOTER_KP.get(), 0, ShooterConstants.SHOOTER_KD.get());

  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
    shooterFeedback.setTolerance(ShooterConstants.SHOOTER_TOLERANCE.get());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    if (Constants.tuningMode) {
      if (ShooterConstants.SHOOTER_KS.hasChanged(hashCode())
          || ShooterConstants.SHOOTER_KD.hasChanged(hashCode())) {
        shooterFeedforward =
            new SimpleMotorFeedforward(
                ShooterConstants.SHOOTER_KS.get(), ShooterConstants.SHOOTER_KV.get());
      }
      if (ShooterConstants.SHOOTER_KP.hasChanged(hashCode())
          || ShooterConstants.SHOOTER_KD.hasChanged(hashCode())) {
        Logger.recordOutput("Shooter/newP", ShooterConstants.SHOOTER_KP.get());
        shooterFeedback.setPID(
            ShooterConstants.SHOOTER_KP.get(), 0, ShooterConstants.SHOOTER_KD.get());
      }
    }
    if (PIDMode) {
      io.setShooterVoltage(
          shooterFeedforward.calculate(currentVelocitySetpoint)
              + shooterFeedback.calculate(
                  inputs.shooterLeaderVelocityRadPerSec, currentVelocitySetpoint));
    }
    Logger.recordOutput("Shooter/setPointVelocity", shooterFeedback.getSetpoint());
    Logger.recordOutput("Shooter/error", shooterFeedback.getVelocityError());
    robotState.shooterSpeedIsReady = ShooterSpeedIsReady();
  }
  /**
   * @param velocitySetpoint the velocity that the shooter should be set to
   * @return A command that sets the PIDMode to true, and then sets to PID setpoint to that of the
   *     inputted velocitySetpoint
   */
  public Command shoot(double velocitySetpoint) {
    return Commands.run(
        () -> {
          PIDMode = true;
          currentVelocitySetpoint = velocitySetpoint;
        },
        this);
  }

  /**
   * @param percent the volts that the shooter should be set to
   * @return A command that sets the shooter voltage to that of the inputed volts
   */
  public Command manualShoot(double percent) {
    return Commands.run(
            () -> {
              io.setShooterVelocity(percent);
              PIDMode = false;
              robotState.shooting = true;
            },
            this)
        .finallyDo(() -> robotState.shooting = false);
  }

  public Command manualShootVolts(double volts) {
    return Commands.run(
        () -> {
          io.setShooterVoltage(volts);
          PIDMode = false;
        },
        this);
  }
  /**
   * @return if the shooter is at the speed required to shoot by checking if the shooters speed is
   *     that of the setpoint of the PID.
   */
  public boolean ShooterSpeedIsReady() {
    return inputs.shooterLeftVelocityRadPerSec
            >= ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC
        && inputs.shooterRightVelocityRadPerSec
            >= ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC;
  }

  public boolean atVelocity(double velocitySetpoint) {
    return inputs.shooterLeftVelocityRadPerSec >= velocitySetpoint - 0.05
        && inputs.shooterRightVelocityRadPerSec >= velocitySetpoint - 0.05;
  }
}
