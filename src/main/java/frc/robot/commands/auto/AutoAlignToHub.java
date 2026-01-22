package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;


public class AutoAlignToHub extends Command {
  public enum BumperSide {
    LEFT,
    RIGHT
  }

  private final Drive drive;
  private final BumperSide bumperSide;
  private Command driveCommand;

  public AutoAlignToHub(Drive drive, BumperSide side) {
    this.drive = drive;
    this.bumperSide = side;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d targetPose = getTargetPose();
    driveCommand = new StraightDriveToPose(drive, targetPose);
    driveCommand.initialize();
  }

  @Override
  public void execute() {
    if (driveCommand != null) {
      driveCommand.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (driveCommand != null) {
      driveCommand.end(interrupted);
    }
  }

  @Override
  public boolean isFinished() {
    return driveCommand != null && driveCommand.isFinished();
  }

 
  private Pose2d getTargetPose() {
    boolean isBlue =
        DriverStation.getAlliance().map(alliance -> alliance == Alliance.Blue).orElse(true);

    if (isBlue) {
      return bumperSide == BumperSide.LEFT
          ? AutoAlignConstants.BLUE_LEFT_BUMPER_POSE
          : AutoAlignConstants.BLUE_RIGHT_BUMPER_POSE;
    } else {
      return bumperSide == BumperSide.LEFT
          ? AutoAlignConstants.RED_LEFT_BUMPER_POSE
          : AutoAlignConstants.RED_RIGHT_BUMPER_POSE;
    }
  }
}
