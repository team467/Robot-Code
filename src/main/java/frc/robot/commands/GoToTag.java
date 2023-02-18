// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GoToTag extends CommandBase {
  /** Creates a new GoToTag. */
  public GoToTag() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // true = left
  public Translation2d leftright(boolean leftright, Translation2d apriltag) {
    if (leftright == true) {
      apriltag = apriltag.plus(new Translation2d(1, 0));
    } else {
      apriltag = apriltag.plus(new Translation2d(-1, 0));
    }
    return apriltag;
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

    var aprilone = aprilOneThreeD.getTranslation().toTranslation2d();
    Translation2d apriloneleft = leftright(true, aprilone);
    Translation2d apriloneright = leftright(false, aprilone);
    Translation2d apriltwo = new Translation2d(aprilTwoThreeD.getX(), aprilTwoThreeD.getY());
    Translation2d apriltwoleft = leftright(true, apriltwo);
    Translation2d apriltworight = leftright(false, apriltwo);
    Translation2d aprilthree = new Translation2d(aprilThreeThreeD.getX(), aprilThreeThreeD.getY());
    Translation2d aprilthreeleft = leftright(true, aprilthree);
    Translation2d aprilthreeright = leftright(false, aprilthree);
    Rotation2d rotationaprilone = new Rotation2d(aprilOneThreeD.getRotation().getAngle());
    Rotation2d rotationapriltwo = new Rotation2d(aprilTwoThreeD.getRotation().getAngle());
    Rotation2d rotationaprilthree = new Rotation2d(aprilThreeThreeD.getRotation().getAngle());
    // left and rights should shift x by a certain messurment
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
