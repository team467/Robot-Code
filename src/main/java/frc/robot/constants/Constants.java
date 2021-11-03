package frc.robot.constants;

import frc.robot.drive.MotorType;

public interface Constants {
    public boolean driveDualMotors();
    public int driveMotorLeftLeaderId();
    public MotorType driveMotorLeftLeaderType();
    public boolean driveMotorLeftLeaderInverted();

    public int driveMotorLeftFollowerId();
    public MotorType driveMotorLeftFollowerType();
    public boolean driveMotorLeftFollowerInverted();

    public int driveMotorRightLeaderId();
    public MotorType driveMotorRightLeaderType();
    public boolean driveMotorRightLeaderInverted();

    public int driveMotorRightFollowerId();
    public MotorType driveMotorRightFollowerType();
    public boolean driveMotorRightFollowerInverted();

    public boolean hasClimber2020();
    public int climber2020MotorId();
    public boolean climber2020MotorInverted();
    public double climber2020UpSpeed();
    public double climber2020DownSpeed();
}
