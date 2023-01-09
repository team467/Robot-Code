package frc.robot.constants;

public class CompBotConstants implements Constants {

  @Override
  public RobotType robot() {
    return RobotType.ROBOT_COMP;
  }

  @Override
  public String logFolder() {
    return "/media/sda1";
  }
}
