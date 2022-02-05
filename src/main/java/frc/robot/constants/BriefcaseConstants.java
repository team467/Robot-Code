package frc.robot.constants;

import frc.robot.motors.MotorType;

public class BriefcaseConstants implements Constants {

    @Override
    public String name() {
        return "Briefcase";
    }

    @Override
    public boolean hasDrivetrain() {
        return false;
    }

    @Override
    public boolean driveDualMotors() {
        return false;
    }

    @Override
    public MotorType driveMotorType() {
        return MotorType.TALON_SRX;
    }

    @Override
    public int driveMotorLeftLeaderId() {
        return 11;
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
        return 12;
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
        return 1;
    }

    @Override
    public boolean climber2020MotorInverted() {
        return false;
    }

    @Override
    public double climber2020UpSpeed() {
        return 0.1;
    }

    @Override
    public double climber2020DownSpeed() {
        return 0.1;
    }

    @Override
    public boolean hasShooter2020() {
        return false;
    }

    @Override
    public MotorType shooter2020MotorType() {
        return MotorType.NONE;
    }

    @Override
    public boolean shooter2020FlywheelDualMotors() {
        return false;
    }

    @Override
    public int shooter2020FlywheelLeaderMotorId() {
        return 0;
    }

    @Override
    public boolean shooter202FlywheelLeaderInverted() {
        return false;
    }

    @Override
    public int shooter2020FlywheelFollowerMotorId() {
        return 0;
    }

    @Override
    public boolean shooter202FlywheelFollowerInverted() {
        return false;
    }

    @Override
    public double shooter2020FlywheelDefaultSpeed() {
        return 0.0;
    }

    @Override
    public boolean shooter2020FlywheelUseVelocity() {
        return false;
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
    public double shooter2020FlywheelkS() {
        return 0.0;
    }

    @Override
    public double shooter2020FlywheelkV() {
        return 0.0;
    }

    @Override
    public double shooter2020FlywheelkA() {
        return 0.0;
    }

    @Override
    public double shooter2020FlywheelkMaxVelocity() {
        return 0;
    }

    @Override
    public int shooter2020TriggerMotorId() {
        return 0;
    }

    @Override
    public boolean shooter2020TriggerInverted() {
        return false;
    }

    @Override
    public int shooter2020LeftServoId() {
        return 0;
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
        return 0;
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
    public boolean hasIndexer2022() {
        return true;
    }

    @Override
    public int indexer2022MotorID() {
        return 11;
    }

    @Override
    public double indexer2022IdleSpeed() {
        return 0.25;
    }

    @Override
    public double indexer2022InSpeed() {
        return 0.5;
    }

    @Override
    public double indexer2022OutSpeed() {
        return 0.5;
    }

    @Override
    public boolean indexer2022MotorInverted() {
        return false;
    }
    
    @Override
    public boolean hasLlamaNeck2022() {
        return true;
    }

    @Override
    public int llamaNeck2022MotorID() {
        return 2;
    }

    @Override
    public double llamaNeck2022IdleSpeed() {
        return 0;
    }

    @Override
    public double llamaNeck2022InSpeed() {
        return 0.5;
    }
    
    @Override
    public double llamaNeck2022OutSpeed() {
     return 0.5;   
    }

    @Override
    public int llamaNeck2022UpperLimitSwitchChannel() {
        return 1;
    }

    @Override
    public int llamaNeck2022LowerLimitSwitchChannel() {
        return 0;
    }

    @Override
    public boolean hasSpitter2022() {
        return true;
    }

    @Override 
    public int spitter2022MotorId() {
        return 1;
    }

    @Override
    public boolean spitter2022MotorInverted() {
        return true;
    }

    @Override
    public double spitter2022kP() {
        return 0;
    }

    @Override
    public double spitter2022kI() {
        return 0;
    }

    @Override
    public double spitter2022kD() {
        return 0;
    }

    @Override
    public double spitter2022kS() {
        return 0;
    }

    @Override
    public double spitter2022kV() {
        return 0;
    }

    @Override
    public double spitter2022kA() {
        return 0;
    }

    @Override
    public boolean spitter2022UsePID() {
        return false;
    }

    @Override
    public double spitter2022ForwardSpeed() {
        return 0.7;
    }

    @Override
    public double spitter2022BackwardSpeed() {
        return 0.5;
    }
    
}
