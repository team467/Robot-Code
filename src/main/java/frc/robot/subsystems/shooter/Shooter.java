// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  /** Creates a new shooter. */
  private final ShooterIO io;

  // private SimpleMotorFeedforward shooterFeedforward =
  //     new SimpleMotorFeedforward(
  //         ShooterConstants.SHOOTER_KS.get(), ShooterConstants.SHOOTER_KV.get());
  private PIDController shooterFeedack =
      new PIDController(ShooterConstants.SHOOTER_KP, 0, ShooterConstants.SHOOTER_KD);

  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command shoot(double velocitySetpoint) {
    return Commands.run(
        () -> {
          shooterFeedack.setSetpoint(velocitySetpoint);
          io.setShooterVoltage(shooterFeedack.calculate(inputs.shooterLeaderVelocityRadPerSec));
        },
        this);
  }

  public boolean getShooterSpeedIsReady(double shooterReadyVelocityRadPerSec) {
    return inputs.shooterLeaderVelocityRadPerSec >= shooterReadyVelocityRadPerSec;
  }
}
