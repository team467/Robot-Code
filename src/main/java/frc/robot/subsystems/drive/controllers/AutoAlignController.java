package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotOdometry;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAlignController {
  @AutoLogOutput(key = "AutoAlign/goalPose")
  private Pose2d goalPose = null;

  // Controllers for translation and rotation
  private final ProfiledPIDController linearController;
  private final ProfiledPIDController thetaController;

  // Store previous velocities for acceleration limiting
  private Translation2d prevLinearVelocity;

  private static final double DRIVE_TOLERANCE = 0.01;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(1.0);

  public AutoAlignController(Pose2d goalPose) {
    this.goalPose = goalPose;
    // Set up both controllers
    linearController =
        new ProfiledPIDController(
            1.0,
            0,
            0,
            new TrapezoidProfile.Constraints(
                DriveConstants.MAX_LINEAR_SPEED, Math.pow(DriveConstants.MAX_LINEAR_SPEED, 2)));
    linearController.setTolerance(DRIVE_TOLERANCE);
    thetaController =
        new ProfiledPIDController(
            1.0,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(360), Units.degreesToRadians(720)));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);

    // Reset measurements and velocities
    Pose2d currentPose = RobotOdometry.getInstance().getLatestPose();
    Twist2d fieldVelocity = RobotOdometry.getInstance().fieldVelocity();
    // Linear controller will control to 0 so distance is the measurement
    Rotation2d rotationToGoal =
        goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    double velocity =
        -new Translation2d(fieldVelocity.dx, fieldVelocity.dy)
            .rotateBy(rotationToGoal.unaryMinus())
            .getX();
    linearController.reset(
        currentPose.getTranslation().getDistance(goalPose.getTranslation()), velocity);
    thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.dtheta);

    // Set goal positions
    linearController.setGoal(0.0);
    thetaController.setGoal(goalPose.getRotation().getRadians());

    // Store linear velocity for acceleration limiting
    prevLinearVelocity = new Translation2d(fieldVelocity.dx, fieldVelocity.dy);

    // Log goal pose
    Logger.recordOutput("AutoAlign/GoalPose", goalPose);
  }

  public ChassisSpeeds update() {
    // Control to setpoint
    Pose2d currentPose = RobotOdometry.getInstance().getLatestPose();

    // Calculate feedback velocities (based on position error).
    double linearVelocityScalar =
        linearController.calculate(
                currentPose.getTranslation().getDistance(goalPose.getTranslation()))
            + linearController.getSetpoint().velocity;
    Rotation2d rotationToGoal =
        goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    Translation2d desiredLinearVelocity = new Translation2d(-linearVelocityScalar, rotationToGoal);

    double angularVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians())
            + thetaController.getSetpoint().velocity;

    ChassisSpeeds fieldRelativeSpeeds =
        new ChassisSpeeds(
            desiredLinearVelocity.getX(), desiredLinearVelocity.getY(), angularVelocity);
    Logger.recordOutput("AutoAlign/FieldRelativeSpeeds", fieldRelativeSpeeds);
    Logger.recordOutput("AutoAlign/LinearError", linearController.getPositionError());
    Logger.recordOutput("AutoAlign/RotationError", thetaController.getPositionError());
    return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, currentPose.getRotation());
  }

  @AutoLogOutput(key = "AutoAlign/AtGoal")
  public boolean atGoal() {
    return linearController.atGoal() && thetaController.atGoal();
  }
}
