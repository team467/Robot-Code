package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  public final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;
  private boolean rotateToTarget = false;

  public Climber(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    if (rotateToTarget) {
      io.goToRotation();
    }
  }

  public boolean atTargetRotation() {
    return inputs.atTargetRotation;
  }

  public Command toRotation(double degrees) {
    return Commands.run(
            () -> {
              rotateToTarget = true;
              io.setRotation(degrees);
            },
            this)
        .until(() -> inputs.atTargetRotation
        ).withName("climberToRotation");
  }

  public Command runPercent(double percent) {
    return Commands.run(
        () -> {
          rotateToTarget = false;
          io.setPercent(percent);
        },
        this).withName("climberRunPercent");
  }

  public double getPositionDegrees() {
    return inputs.positionDegrees;
  }

  public boolean isCalibrated() {
    return inputs.isCalibrated;
  }

  public boolean limitSwitchPressed() {
    return inputs.limitSwitch;
  }
}
