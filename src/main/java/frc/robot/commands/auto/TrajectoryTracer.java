package frc.robot.commands.auto;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import java.util.ArrayList;
import java.util.Timer;
import java.util.function.Supplier;

public class TrajectoryTracer extends Command {
  private final Drive drive;
  private final GyroIO gyroIO;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final ArrayList<WayPoints> waypoints;
  private final ArrayList<Trajectory> subTrajectories;
  private final HolonomicDriveController controller =
      new HolonomicDriveController(
          new PIDController(1, 0, 0),
          new PIDController(1, 0, 0),
          new ProfiledPIDController(
              2.5,
              0,
              0,
              new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720.0))));
  private final GenerateWayPoints generateWayPoints = new GenerateWayPoints();

  private int currentTrajectory;
  private double Timer;

  ;

  public TrajectoryTracer(
      Drive drive,
      Supplier<Pose2d> poseSupplier,
      Supplier<Pose2d> targetPoseSupplier,
      ArrayList<WayPoints> waypoints) {
    this.drive = drive;
    this.poseSupplier = poseSupplier;
    this.targetPoseSupplier = targetPoseSupplier;
    this.waypoints = waypoints;
    this.subTrajectories = new ArrayList<Trajectory>();
    this.gyroIO = new GyroIOPigeon2();
    addRequirements(drive);
    // enable continous output for HolonomicController
    controller.getThetaController().enableContinuousInput(-Math.PI, Math.PI);
    // set tolerances for HolonomicController
    controller.getXController().setTolerance(0.004);
    controller.getYController().setTolerance(0.004);
    controller.getThetaController().setTolerance(Units.degreesToRadians(1.0));
  }

  @Override
  public void initialize() {
    // congifure trajectory for way points and end points
    var configWayPoint = new TrajectoryConfig(Units.feetToMeters(2.16), Units.feetToMeters(2.16));
    var configEndPoint = new TrajectoryConfig(Units.feetToMeters(2.16), Units.feetToMeters(0));
    var configStartPoint = new TrajectoryConfig(Units.feetToMeters(2.16), Units.feetToMeters(2.16));
    generateWayPoints.configure(1);
    configWayPoint.setReversed(true);
    configEndPoint.setReversed(true);
    configStartPoint.setReversed(true);
    currentTrajectory = 0;

    // generate trajectories for way points and end points
    for (int i = 0; i < waypoints.size(); i++) {
      if (i == waypoints.size()-1) {
        subTrajectories.add(
            TrajectoryGenerator.generateTrajectory(
                waypoints.get(i).startPose,
                generateWayPoints.generate(
                    waypoints.get(i).getPose(), waypoints.get(i + 1).getPose()),
                waypoints.get(i + 1).getPose(),
                configEndPoint));
        break;
      } else if (i == 0) {
        subTrajectories.add(
            TrajectoryGenerator.generateTrajectory(
                poseSupplier.get(),
                generateWayPoints.generate(poseSupplier.get(), waypoints.get(i).getPose()),
                waypoints.get(i).getPose(),
                configStartPoint));
      } else {
        subTrajectories.add(
            TrajectoryGenerator.generateTrajectory(
                waypoints.get(i).startPose,
                generateWayPoints.generate(
                    waypoints.get(i).getPose(), waypoints.get(i + 1).getPose()),
                waypoints.get(i + 1).getPose(),
                configWayPoint));
      }
    }
    Timer = 0;
  }

  public void execute() {
    // get current pose and target pose
    Pose2d currentPose = drive.getPose();
    Trajectory neededTrajectory = subTrajectories.get(currentTrajectory);
    // calculate drive speed
    controller.getYController().reset();
    controller.getXController().reset();
    controller.getThetaController().reset(currentPose.getRotation().getRadians());
    ChassisSpeeds calculatedSpeed =
        controller.calculate(
            currentPose, neededTrajectory.sample(Timer), currentPose.getRotation());
    drive.runVelocity(calculatedSpeed);
    if (currentPose.equals(waypoints.get(currentTrajectory).getPose())) {
      currentTrajectory += 1;
      Timer = 0;
    }
    Timer += 0.2;

    // set drive speed
  }
}
