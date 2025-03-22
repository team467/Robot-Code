package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorRelativeToPose extends Command {

  private final Elevator elevator;
  private final Drive drive;
  private final double targetPosition;
  private final double initialPosition;
  private final Pose2d targetPose;
  private Pose2d initialPose;
  private double distanceScaler;
  private double elevatorScaler;
  private double scaler;

  public ElevatorRelativeToPose(
      Elevator elevator, double targetPosition, Pose2d targetPose, Drive drive) {
    this.elevator = elevator;
    this.targetPosition = targetPosition;
    this.targetPose = targetPose;
    this.drive = drive;
    this.initialPose = drive.getPose();
    this.initialPosition = elevator.getPosition();
  }

  @Override
  public void initialize() {
    initialPose = drive.getPose();
    distanceScaler = initialPose.getTranslation().getDistance(targetPose.getTranslation());
    elevatorScaler = targetPosition - initialPosition;
    scaler = elevatorScaler / distanceScaler;
  }

  @Override
  public void execute() {

    if (elevator.getPosition() >= 0.7605) {
      end(false);
    } else {

      Pose2d currentPose = drive.getPose();
      double distance = targetPose.getTranslation().getDistance(currentPose.getTranslation());
      double targetPosition = setpoint(distance);
      elevator.toSetpoint(targetPosition);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.toSetpoint(0.7605);
  }

  public double setpoint(double distance) {
    double setpoint = (0.368) * Math.pow(0.1, distance);
    if (setpoint >= 0.7605) {
      setpoint = 0.7605;
    }
    return setpoint;
  }
}
