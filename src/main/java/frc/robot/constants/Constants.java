package frc.robot.constants;

import frc.robot.motors.MotorType;

public interface Constants {
    public String name();
    public boolean hasDrivetrain();
    public double driveWheelDiameter();
    public boolean driveDualMotors();
    public MotorType driveMotorType();
    public boolean driveUseVelocity();
    public double drivekP();
    public double drivekI();
    public double drivekD();
    public double drivekS();
    public double drivekV();
    public double drivekA();
    public double drivekMaxVelocity();
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

    public boolean hasShooter2020();
    public MotorType shooter2020MotorType();
    public boolean shooter2020FlywheelDualMotors();
    public int shooter2020FlywheelLeaderMotorId();
    public boolean shooter202FlywheelLeaderInverted();
    public int shooter2020FlywheelFollowerMotorId();
    public boolean shooter202FlywheelFollowerInverted();
    public double shooter2020FlywheelDefaultSpeed();
    public boolean shooter2020FlywheelUseVelocity();
    public double shooter2020FlywheelkP();
    public double shooter2020FlywheelkI();
    public double shooter2020FlywheelkD();
    public double shooter2020FlywheelkS();
    public double shooter2020FlywheelkV();
    public double shooter2020FlywheelkA();
    public double shooter2020FlywheelkMaxVelocity();
    public int shooter2020TriggerMotorId();
    public boolean shooter2020TriggerInverted();
    public int shooter2020LeftServoId();
    public double shooter2020LeftServoMax();
    public double shooter2020LeftServoMin();
    public int shooter2020RightServoId();
    public double shooter2020RightServoMax();
    public double shooter2020RightServoMin();
}
