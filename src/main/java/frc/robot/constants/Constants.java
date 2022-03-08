package frc.robot.constants;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.motors.FeedbackConstant;
import frc.robot.motors.GearRatio;
import frc.robot.motors.RamseteConstant;
import frc.robot.motors.SimpleFeedforwardConstant;
import frc.robot.motors.MotorType;
import frc.robot.utilities.IMUAxis;
import frc.robot.utilities.IMUType;

public interface Constants {
    /** @return The name of the robot constant file */
    public String name();
    /** @return Does the robot have a drivetrain */
    public boolean hasDrivetrain();
    /** @return Does the drivetrain use 2 motors */
    public boolean driveDualMotors();
    /** @return What motor type does the drivetrain use */
    public MotorType driveMotorType();
    /** TODO: add explanation to what this is */
    public IdleMode driveIdleMode();
    /** @return Does the drivetrain use velocity tuning */
    public boolean driveUseVelocity();
    /** @return Does the drivetrain use PID tuning */
    public boolean driveUsePID();
    /** TODO: add explanation to what this is */
    public RamseteConstant driveRamsete();
    /** TODO: add explanation to what this is */
    public SimpleFeedforwardConstant driveDriveFF();
    /** TODO: add explanation to what this is */
    public FeedbackConstant driveDriveVelocityPID();
    /** TODO: add explanation to what this is */
    public FeedbackConstant driveDrivePositionPID();
    /** TODO: add explanation to what this is */
    public SimpleFeedforwardConstant driveTurnFF();
    /** TODO: add explanation to what this is */
    public FeedbackConstant driveTurnVelocityPID();
    /** TODO: add explanation to what this is */
    public FeedbackConstant driveTurnPositionPID();
    /** TODO: add explanation to what this is */
    public double driveWheelDiameter();
    /** TODO: add explanation to what this is */
    public GearRatio driveGearRatio();
    /** TODO: add explanation to what this is */
    public DifferentialDriveKinematics driveKinematics();
    public double driveMaxVelocity();
    public double driveMaxAcceleration();
    public double driveAutoMaxVelocity();
    public double driveAutoMaxAcceleration();
    public double driveFastMaxSpeed();
    public double driveNormalMaxSpeed();
    public double driveSlowMaxSpeed();
    public double driveNormalTurnMaxSpeed();
    public double driveSlowTurnMaxSpeed();
    public int driveMotorLeftLeaderId();
    public boolean driveMotorLeftLeaderInverted();
    public int driveMotorLeftFollowerId();
    public boolean driveMotorLeftFollowerInverted();
    public int driveMotorRightLeaderId();
    public boolean driveMotorRightLeaderInverted();
    public int driveMotorRightFollowerId();
    public boolean driveMotorRightFollowerInverted();

    /** @return Does the robot have a gyro */
    public boolean hasGyro();
    /** TODO: add explanation to what this is */
    public IMUType gyroIMUType();
    /** TODO: add explanation to what this is */
    public IMUAxis gyroYawAxis();

    /** @return Does the robot have a 2020 climber */
    public boolean hasClimber2020();
    public int climber2020MotorId();
    public boolean climber2020MotorInverted();
    public double climber2020UpSpeed();
    public double climber2020DownSpeed();

    /** @return Does the robot have a 2022 climber */
    public boolean hasClimber2022();
    /** @return The motor ID for the right motor */
    public int climber2022RightMotorId();
    /** @return The motor ID for the left motor */
    public int climber2022LeftMotorId();
    /** @return Is the left motor inverted */
    public boolean climber2022LeftMotorInverted();
    /** @return Is the right motor inverted */
    public boolean climber2022RightMotorInverted();
    /** @return The speed in which the climber goes up */
    public double climber2022UpSpeed();
    /** @return The speed in which the climber goes down */
    public double climber2022DownSpeed();
    /** TODO: add explanation to what this is */
    public int climber2022SolenoidChannel();
    public double climber2022LeftLowerLimit();
    public double climber2022RightLowerLimit();
    public double climber2022LeftUpperLimit();
    public double climber2022RightUpperLimit();

    /** @return Does the robot have a 2020 shooter */
    public boolean hasShooter2020();
    /** @return What motor type does the shooter use */
    public MotorType shooter2020MotorType();
    /** @return Does the shooter use 2 motors */
    public boolean shooter2020FlywheelDualMotors();
    public int shooter2020FlywheelLeaderMotorId();
    public boolean shooter202FlywheelLeaderInverted();
    public int shooter2020FlywheelFollowerMotorId();
    public boolean shooter202FlywheelFollowerInverted();
    public double shooter2020FlywheelDefaultSpeed();
    public boolean shooter2020FlywheelUseVelocity();
    /** TODO: add explanation to what this is */
    public double shooter2020FlywheelkP();
    /** TODO: add explanation to what this is */
    public double shooter2020FlywheelkI();
    /** TODO: add explanation to what this is */
    public double shooter2020FlywheelkD();
    /** TODO: add explanation to what this is */
    public double shooter2020FlywheelkS();
    /** TODO: add explanation to what this is */
    public double shooter2020FlywheelkV();
    /** TODO: add explanation to what this is */
    public double shooter2020FlywheelkA();
    public double shooter2020FlywheelkMaxVelocity();
    public int shooter2020TriggerMotorId();
    public boolean shooter2020TriggerInverted();
    /** TODO: add explanation to what this is */
    public int shooter2020LeftServoId();
    /** TODO: add explanation to what this is */
    public double shooter2020LeftServoMax();
    /** TODO: add explanation to what this is */
    public double shooter2020LeftServoMin();
    /** TODO: add explanation to what this is */
    public int shooter2020RightServoId();
    /** TODO: add explanation to what this is */
    public double shooter2020RightServoMax();
    /** TODO: add explanation to what this is */
    public double shooter2020RightServoMin();

    /** @return Does the robot have a 2022 indexer */
    public boolean hasIndexer2022();
    public int indexer2022MotorID();
    public double indexer2022IdleSpeed();
    public double indexer2022InSpeed();
    public double indexer2022OutSpeed();
    public boolean indexer2022MotorInverted();

    /** @return Does the robot have a 2022 llama neck */
    public boolean hasLlamaNeck2022();
    public int llamaNeck2022MotorID();
    public boolean llamaNeck2022MotorInverted();
    public double llamaNeck2022IdleSpeed();
    public double llamaNeck2022InSpeed();
    public double llamaNeck2022OutSpeed();
    public int llamaNeck2022UpperLimitSwitchChannel();
    public int llamaNeck2022LowerLimitSwitchChannel();

    /** @return Does the robot have a 2022 spitter */
    public boolean hasSpitter2022();
    public int spitter2022MotorId();
    public boolean spitter2022MotorInverted();
    public boolean spitter2022UseVelocity();
    public boolean spitter2022UsePID();
    /** TODO: add explanation to what this is */
    public SimpleFeedforwardConstant spitter2022FF();
    /** TODO: add explanation to what this is */
    public double spitter2022MomentOfInertia();
    /** TODO: add explanation to what this is */
    public GearRatio spitter2022GearRatio();
    /** TODO: add explanation to what this is */
    public FeedbackConstant spitter2022FB();
    public double spitter2022MaxVelocity();
    public double spitter2022ForwardSpeed();
    public double spitter2022BackwardSpeed();
    /** TODO: add explanation to what this is */
    public double spitter2022DistanceLinearM();
    /** TODO: add explanation to what this is */
    public double spitter2022DistanceLinearB();

    /** @return Does the robot have a hub camera LED */
    public boolean hasHubCameraLED();
    public int hubCameraLEDChannel();
    public Translation2d hubCameraOffset();
}
