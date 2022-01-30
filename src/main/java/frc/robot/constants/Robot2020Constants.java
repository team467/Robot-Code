package frc.robot.constants;

import frc.robot.motors.MotorType;

public class Robot2020Constants implements Constants {

    @Override
    public String name() {
        return "Robot 2020";
    }

    @Override
    public boolean hasDrivetrain() {
        return false;
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
    public boolean hasClimber2022() {
        return true;
    }

    @Override
    public int climber2022MotorId() {
        return 5;
    }

    @Override
    public boolean climber2022MotorInverted() {
        return false;
    }

    @Override
    public double climber2022UpSpeed() {
        return 1.0;
    }

    @Override
    public double climber2022DownSpeed() {
        return 0.8;
    }

    @Override
    public boolean hasShooter2020() {
        return true;
    }

    @Override
    public MotorType shooter2020MotorType() {
        return MotorType.SPARK_MAX_BRUSHLESS;
    }

    @Override
    public boolean shooter2020FlywheelDualMotors() {
        return true;
    }

    @Override
    public int shooter2020FlywheelLeaderMotorId() {
        return 1;
    }

    @Override
    public boolean shooter202FlywheelLeaderInverted() {
        return false;
    }

    @Override
    public int shooter2020FlywheelFollowerMotorId() {
        return 2;
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
        return 0.0248;
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
    public double shooter2020FlywheelkS() {
        return -0.143;
    }

    @Override
    public double shooter2020FlywheelkV() {
        return 0.13;
    }

    @Override
    public double shooter2020FlywheelkA() {
        return 0.0062;
    }

    @Override
    public double shooter2020FlywheelkMaxVelocity() {
        return 80.0;
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

    @Override
    public boolean hasTrigger2022() {
        return false;
    }

    @Override
    public int trigger2022MotorID() {
        return 0;
    }

    @Override
    public double trigger2022IdleSpeed() {
        return 0;
    }

    @Override
    public double trigger2022InSpeed() {
        return 0;
    }

    @Override
    public double trigger2022OutSpeed() {
        return 0;
    }

    @Override
    public boolean hasLlamaNeck2022() {
        return false;
    }

    @Override
    public int llamaNeck2022MotorID() {
        return 0;
    }

    @Override
    public double llamaNeck2022InSpeed() {
        return 0;
    }
    
    @Override
    public double llamaNeck2022OutSpeed() {
     return 0;   
    }

    @Override
    public int llamaNeck2022UpperLimitSwitchChannel() {
        return 0;
    }

    @Override
    public int llamaNeck2022LowerLimitSwitchChannel() {
        return 0;
    }

    @Override
    public boolean hasIntake2022() {
        return false;
    }
    
    @Override
    public int intake2022MotorID() {
        return 0;
    }

    @Override
    public double intake2022InSpeed() {
        return 0;
    }

    @Override
    public double intake2022OutSpeed() {
        return 0;
    }
}
