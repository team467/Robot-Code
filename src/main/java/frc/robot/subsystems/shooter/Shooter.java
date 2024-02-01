// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

  private SimpleMotorFeedforward shooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.SHOOTER_KS.get(), ShooterConstants.SHOOTER_KV.get());
  private PIDController shooterFeedack =
      new PIDController(ShooterConstants.SHOOTER_KP.get(), 0, ShooterConstants.SHOOTER_KD.get());

  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/setPointVelocity", shooterFeedack.getSetpoint());
    Logger.recordOutput("Shooter/error", shooterFeedack.getVelocityError());
    if (Constants.tuningMode) {
      if (ShooterConstants.SHOOTER_KP.hasChanged(hashCode())
          || ShooterConstants.SHOOTER_KD.hasChanged(hashCode())) {
        Logger.recordOutput("Shooter/newP", ShooterConstants.SHOOTER_KP.get());
        shooterFeedack.setPID(
            ShooterConstants.SHOOTER_KP.get(), 0, ShooterConstants.SHOOTER_KD.get());
      }
    }
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setShooterVoltage(0.0);
        },
        this);
  }

  public Command shootFeedFoward(double velocitySetpoint) {
    return Commands.run(() -> io.setShooterVoltage(shooterFeedforward.calculate(velocitySetpoint)));
  }

  public Command shoot(double velocitySetpoint) {
    return Commands.run(
        () -> {
          io.setShooterVoltage(
              shooterFeedack.calculate(inputs.shooterLeaderVelocityRadPerSec, velocitySetpoint));
        },
        this);
  }

  public Command manualShoot(double volts) {
    return Commands.run(() -> io.setShooterVoltage(volts), this);
  }

  public boolean getShooterSpeedIsReady() {
    if (shooterFeedack.getSetpoint() == 0) {
      return false;
    } else {
      return shooterFeedack.atSetpoint();
    }
  }
}
