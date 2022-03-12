package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.RobotPose2022;
import org.apache.logging.log4j.Logger;

public class RobotPose2022UpdateWithGyroCMD extends CommandBase {
    private static final Logger LOGGER =
        RobotLogManager.getMainLogger(RobotPose2022AngleToTargetCMD.class.getName());
    private final RobotPose2022 robotPose;


  public RobotPose2022UpdateWithGyroCMD(RobotPose2022 robotPose) {
    this.robotPose = robotPose;
    addRequirements(robotPose);
  }

  @Override
  public void initialize() {
    LOGGER.debug("Updating robot's angle through gyro");
  }

  @Override
  public void execute() {
    robotPose.updateWithGyro();
    //LOGGER.debug(angleToTarget); 
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
