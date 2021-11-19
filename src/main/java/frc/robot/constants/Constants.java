package frc.robot.constants;

import frc.robot.drive.MotorType;

public interface Constants {
    public boolean driveDualMotors();
    public MotorType driveMotorType();
    public int driveMotorLeftLeaderId();
    public boolean driveMotorLeftLeaderInverted();
    public int driveMotorLeftFollowerId();
    public boolean driveMotorLeftFollowerInverted();
    public int driveMotorRightLeaderId();
    public boolean driveMotorRightLeaderInverted();
    public int driveMotorRightFollowerId();
    public boolean driveMotorRightFollowerInverted();

    public boolean hasClimber2020();
    public int climber2020MotorId();
    public boolean climber2020MotorInverted();
    public double climber2020UpSpeed();
    public double climber2020DownSpeed();

    public boolean hasIntake2020();
    public int intake2020RollerMotorID();
    public int intake2020ArmMotorID();
    public double intake2020RollerForwardSpeed();
    public double intake2020RollerBackwardSpeed();
    public double intake2020ArmDownSpeed();
    public double intake2020ArmUpSpeed();
}
