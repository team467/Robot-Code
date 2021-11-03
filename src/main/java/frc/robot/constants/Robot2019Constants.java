package frc.robot.constants;

import frc.robot.drive.MotorType;

public class Robot2019Constants implements Constants {

    @Override
    public boolean driveDualMotors() {
        return true;
    }

    @Override
    public int driveMotorLeftLeaderId() {
        return 1;
    }

    @Override
    public MotorType driveMotorLeftLeaderType() {
        return MotorType.TALON_SRX;
    }

    @Override
    public boolean driveMotorLeftLeaderInverted() {
        return false;
    }

    @Override
    public int driveMotorLeftFollowerId() {
        return 2;
    }

    @Override
    public MotorType driveMotorLeftFollowerType() {
        return MotorType.TALON_SRX;
    }

    @Override
    public boolean driveMotorLeftFollowerInverted() {
        return false;
    }

    @Override
    public int driveMotorRightLeaderId() {
        return 3;
    }

    @Override
    public MotorType driveMotorRightLeaderType() {
        return MotorType.TALON_SRX;
    }

    @Override
    public boolean driveMotorRightLeaderInverted() {
        return false;
    }

    @Override
    public int driveMotorRightFollowerId() {
        return 4;
    }

    @Override
    public MotorType driveMotorRightFollowerType() {
        return MotorType.TALON_SRX;
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
        return 0.3;
    }

    @Override
    public double climber2020DownSpeed() {
        return 0.1;
    }
    
}
