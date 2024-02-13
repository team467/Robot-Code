package frc.robot.subsystems.drive.controllers.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.AutoAlignController;

public class AutoAlign extends Command {
  private final AutoAlignController controller;
  private final Drive drive;

  public AutoAlign(Drive drive, Pose2d goalPose) {
    controller = new AutoAlignController(goalPose);
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    ChassisSpeeds controlSpeeds = controller.update();
    drive.runVelocity(controlSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stopWithX();
  }

  @Override
  public boolean isFinished() {
    return controller.atGoal();
  }
}
