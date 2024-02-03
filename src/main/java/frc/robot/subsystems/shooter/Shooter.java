package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  /** Creates a new shooter. */
  private final ShooterIO io;

  private boolean PIDMode = false;
  private double currentVelocitySetpoint;
  private static double speakerHight = 211.0;
  private SimpleMotorFeedforward shooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.SHOOTER_KS.get(), ShooterConstants.SHOOTER_KV.get());
  private PIDController shooterFeedack =
      new PIDController(ShooterConstants.SHOOTER_KP.get(), 0, ShooterConstants.SHOOTER_KD.get());

  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
    shooterFeedack.setTolerance(ShooterConstants.SHOOTER_TOLERANCE.get());
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
        shooterFeedack.setPID(
            ShooterConstants.SHOOTER_KP.get(), 0, ShooterConstants.SHOOTER_KD.get());
      }
    }
    if (PIDMode) {
      io.setShooterVoltage(
          shooterFeedforward.calculate(currentVelocitySetpoint)
              + shooterFeedack.calculate(
                  inputs.shooterLeaderVelocityRadPerSec, currentVelocitySetpoint));
    }
    Logger.recordOutput("Shooter/setPointVelocity", shooterFeedack.getSetpoint());
    Logger.recordOutput("Shooter/error", shooterFeedack.getVelocityError());
  }
  /**
   * @param velocitySetpoint
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
   * @param volts
   * @return A command that sets the shooter voltage to that of the inputed volts
   */
  public Command manualShoot(double volts) {
    return Commands.run(() -> io.setShooterVoltage(volts), this);
  }
  /**
   * @return if the shooter is at the speed required to shoot by checking if the shooters speed is
   *     that of the setpoint of the PID.
   */
  public boolean ShooterSpeedIsReady() {
    if (shooterFeedack.getSetpoint() == 0) {
      return false;
    } else {
      return shooterFeedack.atSetpoint();
    }
  }
  /**
   * @param distanceFromSpeaker
   * @return calculates the hypotenuse of the hight of the speaker and the inputed distance
   */
  public double calculateShootingDistance(double distanceFromSpeaker) {
    return Math.hypot(speakerHight, distanceFromSpeaker);
  }

  public double calculateShootingAngle(double distanceFromSpeaker) {
    return Math.abs(Math.atan(speakerHight / distanceFromSpeaker));
  }
}
