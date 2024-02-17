package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.robotstate.RobotState;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  /** Creates a new shooter. */
  private final ShooterIO io;

  private final RobotState robotState = RobotState.getInstance();
  private boolean PIDMode = false;
  private double currentVelocitySetpoint;
  private static final double SPEAKER_HEIGHT = 211.0;
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
                  inputs.shooterLeftVelocityRadPerSec, currentVelocitySetpoint));
    }
    Logger.recordOutput("Shooter/setPointVelocity", shooterFeedback.getSetpoint());
    Logger.recordOutput("Shooter/error", shooterFeedback.getVelocityError());
  }
  /**
   * @param velocitySetpoint the velocity that the shooter should be set to
   * @return A command that sets the PIDMode to true, and then sets to PID setpoint to that of the
   *     inputed velocitySetpoint
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
   * @param volts the volts that the shooter should be set to
   * @return A command that sets the shooter voltage to that of the inputed volts
   */
  public Command manualShoot(double volts) {
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
    if (shooterFeedback.getSetpoint() == 0) {
      return false;
    } else {
      return shooterFeedback.atSetpoint();
    }
  }
  /**
   * @param distanceFromSpeaker the robots distance from the speaker
   * @return calculates the hypotenuse of the hight of the speaker and the inputed distance
   */
  public double calculateShootingDistance(double distanceFromSpeaker) {
    return Math.hypot(SPEAKER_HEIGHT, distanceFromSpeaker);
  }
  /**
   * @param distanceFromSpeaker the robots distance from the speaker
   * @return the angle at which the shooter must be to shoot into the speaker
   */
  public double calculateShootingAngle(double distanceFromSpeaker) {
    return Math.abs(Math.atan(SPEAKER_HEIGHT / distanceFromSpeaker));
  }
}
