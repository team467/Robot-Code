// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GoToTag extends CommandBase {
  /** Creates a new GoToTag. */
  public GoToTag() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // these are FAKE values !
    Pose3d aprilOneThreeD = new Pose3d(7.0, 8.0, 1.0, new Rotation3d(0, 0, 90.0));
    Pose3d aprilTwoThreeD = new Pose3d(7.0, 1.0, 1.0, new Rotation3d(0, 0, 27.0));
    Pose3d aprilThreeThreeD = new Pose3d(1.0, 1.0, 1.0, new Rotation3d(0, 0, 9.0));

    Pose2d aprilOne = new Pose2d(aprilOneThreeD.getX(), aprilOneThreeD.getY(),
        new Rotation2d(aprilOneThreeD.getRotation().getAngle()));
    Pose2d aprilOneLeft = new Pose2d(aprilOneThreeD.getX() + 1, aprilOneThreeD.getY(),
        new Rotation2d(aprilOneThreeD.getRotation().getAngle()));
    Pose2d aprilOneRight = new Pose2d(aprilOneThreeD.getX() - 1, aprilOneThreeD.getY(),
        new Rotation2d(aprilOneThreeD.getRotation().getAngle()));
      
    Pose2d aprilTwo = new Pose2d(aprilTwoThreeD.getX(), aprilTwoThreeD.getY(),
        new Rotation2d(aprilTwoThreeD.getRotation().getAngle()));
    Pose2d aprilTwoLeft = new Pose2d(aprilTwoThreeD.getX() + 1, aprilTwoThreeD.getY(),
        new Rotation2d(aprilTwoThreeD.getRotation().getAngle()));
    Pose2d aprilTwoRight = new Pose2d(aprilTwoThreeD.getX() - 1, aprilTwoThreeD.getY(),
        new Rotation2d(aprilTwoThreeD.getRotation().getAngle()));
    
    Pose2d aprilThree = new Pose2d(aprilThreeThreeD.getX(), aprilThreeThreeD.getY(),
        new Rotation2d(aprilThreeThreeD.getRotation().getAngle()));
    Pose2d aprilThreeLeft = new Pose2d(aprilThreeThreeD.getX() + 1, aprilThreeThreeD.getY(),
        new Rotation2d(aprilThreeThreeD.getRotation().getAngle()));
    Pose2d aprilThreeRight = new Pose2d(aprilThreeThreeD.getX() - 1, aprilThreeThreeD.getY(),
        new Rotation2d(aprilThreeThreeD.getRotation().getAngle()));

      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
