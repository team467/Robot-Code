package frc.robot.constants;

import frc.robot.drive.MotorType;

public class BlankConstants implements Constants {

    @Override
    public boolean driveDualMotors() {
        return false;
    }

    @Override
    public MotorType driveMotorType() {
        return MotorType.NONE;
    }

    @Override
    public int driveMotorLeftLeaderId() {
        return 0;
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
    public boolean driveMotorLeftFollowerInverted() {
        return false;
    }

    @Override
    public int driveMotorRightLeaderId() {
        return 0;
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

    @Override
    public boolean hasIntake2020() {
        return false;
    }

    @Override
    public int intake2020RollerMotorID() {
        return 0; //left from staring thru the power switch
    }

    @Override
    public int intake2020ArmMotorID() {
        return 0; //right from staring thru the power switch
    }

    @Override
    public double intake2020RollerForwardSpeed() {
        return 0;
    }

    @Override
    public double intake2020RollerBackwardSpeed() {
        return 0;
    }

    @Override
    public double intake2020ArmDownSpeed() {
        return 0;
    }

    @Override
    public double intake2020ArmUpSpeed() {
        return 0;
    }
    
}
