package frc.robot.constants;

import frc.robot.motors.MotorType;

public class Robot2020Constants implements Constants {

    @Override
    public boolean hasDrivetrain() {
        return true;
    }

    @Override
    public boolean driveDualMotors() {
        return true;
    }

    @Override
    public MotorType driveMotorType() {
        return MotorType.SPARK_MAX_BRUSHLESS;
    }

    @Override
    public int driveMotorLeftLeaderId() {
        return 1;
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
    public boolean driveMotorLeftFollowerInverted() {
        return false;
    }

    @Override
    public int driveMotorRightLeaderId() {
        return 3;
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
    public boolean driveMotorRightFollowerInverted() {
        return false;
    }

    @Override
    public boolean hasClimber2020() {
        return true;
    }

    @Override
    public int climber2020MotorId() {
        return 5;
    }

    @Override
    public boolean climber2020MotorInverted() {
        return false;
    }

    @Override
    public double climber2020UpSpeed() {
        return 1.0;
    }

    @Override
    public double climber2020DownSpeed() {
        return 0.8;
    }

    @Override
    public boolean hasShooter2020() {
        return true;
    }

    @Override
    public MotorType shooter2020MotorType() {
        return MotorType.TALON_SRX;
    }

    @Override
    public boolean shooter2020FlywheelDualMotors() {
        return true;
    }

    @Override
    public int shooter2020FlywheelLeaderMotorId() {
        return 5;
    }

    @Override
    public boolean shooter202FlywheelLeaderInverted() {
        return true;
    }

    @Override
    public int shooter2020FlywheelFollowerMotorId() {
        return 6;
    }

    @Override
    public boolean shooter202FlywheelFollowerInverted() {
        return false;
    }

    @Override
    public double shooter2020FlywheelDefaultSpeed() {
        return 0.4;
    }

    @Override
    public boolean shooter2020FlywheelUseVelocity() {
        return true;
    }

    @Override
    public double shooter2020FlywheelkP() {
        return 0.0;
    }

    @Override
    public double shooter2020FlywheelkI() {
        return 0.0;
    }

    @Override
    public double shooter2020FlywheelkD() {
        return 0.0;
    }

    @Override
    public double shooter2020FlywheelkF() {
        return 0.123;
    }

    @Override
    public double shooter2020FlywheelkMaxVelocity() {
        return 6000.0;
    }

    @Override
    public int shooter2020TriggerMotorId() {
        return 7;
    }

    @Override
    public boolean shooter2020TriggerInverted() {
        return true;
    }

    @Override
    public int shooter2020LeftServoId() {
        return 2;
    }

    @Override
    public double shooter2020LeftServoMax() {
        return 0;
    }

    @Override
    public double shooter2020LeftServoMin() {
        return 0;
    }

    @Override
    public int shooter2020RightServoId() {
        return 3;
    }

    @Override
    public double shooter2020RightServoMax() {
        return 0;
    }

    @Override
    public double shooter2020RightServoMin() {
        return 0;
    }
}
