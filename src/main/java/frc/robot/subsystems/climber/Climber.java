package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  public final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  public Climber(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    io.goToRotation();
  }

  public boolean atTargetRotation() {
    return inputs.atTargetRotation;
  }

  public Command toRotation(double degrees) {
    return Commands.run(
        () -> {
          io.setRotation(degrees);
        },
        this);
  }

  public Command toRotation(DoubleSupplier degrees) {
    return Commands.run(
        () -> {
          io.setRotation(degrees.getAsDouble());
        },
        this);
  }

  public Command runPercent(double percent) {
    return Commands.run(
        () -> {
          io.setPercent(percent);
        },
        this);
  }

  public double getPositionDegrees() {
    return inputs.positionDegrees;
  }
}
