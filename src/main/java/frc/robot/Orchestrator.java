package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.RobotState.ElevatorPosition;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Orchestrator {
  private final Drive drive;
  private final RobotState robotState = RobotState.getInstance();

  public Orchestrator(
      Drive drive) {
    this.drive = drive;
    robotState.elevatorPosition = ElevatorPosition.INTAKE;
  }
}
