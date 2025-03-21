package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
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

  private final PIDController elevatorController = new PIDController(8, 0, .01);

  @Override
  public void initialize() {
    elevatorController.reset();
    initialPose = drive.getPose();
    distanceScaler = initialPose.getTranslation().getDistance(targetPose.getTranslation());
    elevatorScaler = targetPosition - initialPosition;
    scaler = elevatorScaler / distanceScaler;
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    double distance = targetPose.getTranslation().getDistance(currentPose.getTranslation());
    if (elevatorController.atSetpoint()) {
      end(true);
      distance = 0;
    }
    double output = elevatorController.calculate(distance);
    elevator.toSetpoint(targetPosition + output * scaler);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
