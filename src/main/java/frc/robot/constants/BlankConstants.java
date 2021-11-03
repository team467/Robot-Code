package frc.robot.constants;

import frc.robot.drive.MotorType;

public class BlankConstants implements Constants {

    @Override
    public boolean driveDualMotors() {
        return false;
    }

    @Override
    public int driveMotorLeftLeaderId() {
        return 0;
    }

    @Override
    public MotorType driveMotorLeftLeaderType() {
        return MotorType.NONE;
    }

    @Override
    public boolean driveMotorLeftLeaderInverted() {
        return false;
    }

    @Override
    public int driveMotorLeftFollowerId() {
        return 0;
    }

    @Override
    public MotorType driveMotorLeftFollowerType() {
        return MotorType.NONE;
    }

    @Override
    public boolean driveMotorLeftFollowerInverted() {
        return false;
    }

    @Override
    public int driveMotorRightLeaderId() {
        return 0;
    }

    @Override
    public MotorType driveMotorRightLeaderType() {
        return MotorType.NONE;
    }

    @Override
    public boolean driveMotorRightLeaderInverted() {
        return false;
    }

    @Override
    public int driveMotorRightFollowerId() {
        return 0;
    }

    @Override
    public MotorType driveMotorRightFollowerType() {
        return MotorType.NONE;
    }

    @Override
    public boolean driveMotorRightFollowerInverted() {
        return false;
    }

    @Override
    public boolean hasClimber2020() {
        return false;
    }

    @Override
    public int climber2020MotorId() {
        return 0;
    }

    @Override
    public boolean climber2020MotorInverted() {
        return false;
    }

    @Override
    public double climber2020UpSpeed() {
        return 0;
    }

    @Override
    public double climber2020DownSpeed() {
        return 0;
    }
    
}
